#pragma once

#include "CustomLR1110.h"
#include "RadioLibWrappers.h"
#include "LR11x0Reset.h"

class CustomLR1110Wrapper : public RadioLibWrapper {
  // RX duty cycle: no SPI access while the radio sleeps between windows (it would wake up and leave the cycle).
  // The noise floor is measured in short continuous-receive bursts instead (first at start-up, then every 10 min).
  static const uint32_t FLOOR_RECAL_MS = 600000UL;
  static const uint16_t FLOOR_SAMPLES = 64;   // NUM_NOISE_FLOOR_SAMPLES in RadioLibWrappers.cpp
  bool _calibrating = false;
  bool _floorKnown = false;
  uint32_t _floorAt = 0;

  CustomLR1110* lr() const { return (CustomLR1110*)_radio; }
  bool dutyQuiet() const { return lr()->isDutyActive() && !_calibrating; }   // no polling allowed now

public:
  CustomLR1110Wrapper(CustomLR1110& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void loop() override {
    if (lr()->getRxDutyCycle() && !_calibrating && (!_floorKnown || millis() - _floorAt >= FLOOR_RECAL_MS)) {
      _calibrating = true;                  // continuous receive for one noise-floor round
      lr()->setRxDutyCycle(false);
      if (isInRecvMode()) startRecv();
      _num_floor_samples = 0;
      _floor_sample_sum = 0;
    }
    RadioLibWrapper::loop();
    // calibrating: take the samples back to back (one per loop pass would keep the radio awake for many
    // seconds on a board whose CPU sleeps between passes)
    for (int i = 0; _calibrating && isInRecvMode() && _num_floor_samples < FLOOR_SAMPLES && i < FLOOR_SAMPLES; i++)
      RadioLibWrapper::loop();
    if (_calibrating && _num_floor_samples >= FLOOR_SAMPLES && _floor_sample_sum != 0)
      RadioLibWrapper::loop();              // computes the floor
    if (_calibrating && _num_floor_samples >= FLOOR_SAMPLES && _floor_sample_sum == 0) {
      _calibrating = false;                 // floor computed: back to the duty cycle
      _floorKnown = true;
      _floorAt = millis();
      lr()->setRxDutyCycle(true);
      if (isInRecvMode()) startRecv();
    }
  }

  // RX duty cycle on/off (applied at the next startReceive(); a noise-floor round runs first when enabling)
  void setRxDutyCycle(bool en) {
    _calibrating = false;
    _floorKnown = !en ? _floorKnown : false;
    lr()->setRxDutyCycle(en);
    idle();
  }
  bool getRxDutyCycle() const { return lr()->getRxDutyCycle() || _calibrating; }
  bool isDutyActive() const { return lr()->isDutyActive(); }
  uint32_t getDutyRxUs() const { return lr()->getDutyRxUs(); }
  uint32_t getDutySleepUs() const { return lr()->getDutySleepUs(); }
  // listening share of the cycle, per mille (0 when receiving continuously)
  uint16_t getRxDutyShare() const {
    uint32_t rx = lr()->getDutyRxUs(), sl = lr()->getDutySleepUs();
    return lr()->isDutyActive() && rx + sl ? (uint16_t)((uint64_t)rx * 1000 / (rx + sl)) : 0;
  }

  int recvRaw(uint8_t* bytes, int sz) override {
    // packet received in duty cycle: the radio went back to sleep, wake it before RadioLib reads the packet
    if (lr()->isDutyActive() && lr()->irqPending()) lr()->wakeIfAsleep();
    return RadioLibWrapper::recvRaw(bytes, sz);
  }
  uint32_t getRadioWakes() const { return lr()->getWakes(); }

  bool startSendRaw(const uint8_t* bytes, int len) override {
    lr()->leaveDuty();                      // stage the packet in standby, not across duty-cycle sleeps
    return RadioLibWrapper::startSendRaw(bytes, len);
  }

  // Before transmitting: asleep in its cycle (BUSY high) means nothing is being received, without SPI
  bool isReceiving() override {
    if (lr()->isDutyActive()) {
      if (!lr()->dutyAsleep() && lr()->dutyActivity()) return true;
      return isChannelActive();
    }
    return RadioLibWrapper::isReceiving();
  }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomLR1110 *)_radio)->leaveDuty();
    ((CustomLR1110 *)_radio)->setFrequency(freq);
    ((CustomLR1110 *)_radio)->setSpreadingFactor(sf);
    ((CustomLR1110 *)_radio)->setBandwidth(bw);
    ((CustomLR1110 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
    PacketMillis pm = calcMaxPacketMillis(sf, bw, cr, preambleLengthForSF(sf));
    ((CustomLR1110 *)_radio)->setPreambleMillis(pm.preambleMillis);
    ((CustomLR1110 *)_radio)->setMaxPayloadMillis(pm.payloadMillis);
  }

  bool lastRxHadCRC() override {
    bool crc = true;
    ((CustomLR1110 *)_radio)->getLoRaRxHeaderInfo(NULL, &crc);
    return crc;
  }
  bool isReceivingPacket() override {
    if (dutyQuiet()) return false;          // noise-floor sampling / AGC reset: no SPI during the duty cycle
    return ((CustomLR1110 *)_radio)->isReceiving();
  }
  float getCurrentRSSI() override {
    if (dutyQuiet()) return _noise_floor;   // last measured floor (int.thresh compares against it)
    float rssi = -110;
    ((CustomLR1110 *)_radio)->getRssiInst(&rssi);
    return rssi;
  }

  uint32_t getEstAirtimeFor(int len_bytes) override {
    auto airtime = RadioLibWrapper::getEstAirtimeFor(len_bytes);
    return airtime < 200 ? 200 : airtime;   // at least 200 millis
  }

  void onSendFinished() override {
    RadioLibWrapper::onSendFinished();
    _radio->setPreambleLength(preambleLengthForSF(getSpreadingFactor())); // overcomes weird issues with small and big pkts
  }

  float getLastRSSI() const override { return ((CustomLR1110 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomLR1110 *)_radio)->getSNR(); }

  uint8_t getSpreadingFactor() const override { return ((CustomLR1110 *)_radio)->getSpreadingFactor(); }
  
  bool supportsRxBoostedGain() const override { return true; }
  bool setRxBoostedGainMode(bool en) override {
    return ((CustomLR1110 *)_radio)->setRxBoostedGainMode(en) == RADIOLIB_ERR_NONE;
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomLR1110 *)_radio)->getRxBoostedGainMode();
  }

  void doResetAGC() override { lr11x0ResetAGC((LR11x0 *)_radio, ((CustomLR1110 *)_radio)->getFreqMHz(), getRxBoostedGainMode()); }
};

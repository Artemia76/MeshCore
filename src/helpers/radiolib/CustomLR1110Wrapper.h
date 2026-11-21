#pragma once

#include "CustomLR1110.h"
#include "RadioLibWrappers.h"
#include "LR11x0Reset.h"

class CustomLR1110Wrapper : public RadioLibWrapper {
  CustomLR1110* lr() const { return (CustomLR1110*)_radio; }
#ifdef LR11X0_RX_DUTY_CYCLE
  // RX duty cycle: no SPI access while the radio sleeps between windows (it would wake up and leave the cycle).
  // The noise floor is measured in short continuous-receive bursts instead (first at start-up, then every 10 min).
  static const uint32_t FLOOR_RECAL_MS = 600000UL;
  static const uint16_t FLOOR_SAMPLES = 64;   // NUM_NOISE_FLOOR_SAMPLES in RadioLibWrappers.cpp
  bool _calibrating = false;
  bool _floorKnown = false;
  uint32_t _floorAt = 0;
  // Duty cycle held off (continuous receive) while the companion app is connected. The cycle is sized for 32-symbol
  // preambles (MeshCore >= 1.16 at SF <= 8): 16-symbol frames from older repeaters last about one sleep period, and
  // 21 % (strong) to 52 % (weak signal) of them are missed. Active use (Mesh Mapper, chat) gets full reception;
  // the cycle saves energy at rest. _dutyWanted is the user setting (console "rxduty").
  bool _dutyWanted = LR11X0_RX_DUTY_CYCLE;
  bool _dutyHeld = false;
  void applyDuty() {
    bool en = _dutyWanted && !_dutyHeld;
    if (en == (lr()->getRxDutyCycle() || _calibrating)) return;
    _calibrating = false;
    lr()->setRxDutyCycle(en);
    idle();                                 // next recvRaw() restarts receive in the new mode
  }

  bool dutyQuiet() const { return lr()->isDutyActive() && !_calibrating; }   // no polling allowed now

  // Radio stuck asleep: in duty cycle BUSY is high only between listening windows (tens of ms). On 2026-11-18 the
  // LR1110 stayed asleep (BUSY high, no IRQ on DIO1) for many minutes, deaf, and every command waited 1 s for it.
  // Once BUSY has not been seen low for STUCK_SUSPECT_MS, watch the pin for two cycle periods; still high: wake the
  // radio and restart receive. The IRQ flags found at that moment are kept to find out what put it to sleep.
  static const uint32_t STUCK_SUSPECT_MS = 2000;
  uint32_t _busyLowAt = 0, _stuckWakes = 0, _stuckIrq = 0, _stuckAt = 0;

  void checkStuckAsleep() {
    uint32_t now = millis();
    if (!lr()->isDutyActive() || _calibrating || !isInRecvMode() || lr()->irqPending() || !lr()->dutyAsleep()) {
      _busyLowAt = now;           // not cycling, packet waiting (recvRaw wakes it), or awake right now
      return;
    }
    if (now - _busyLowAt < STUCK_SUSPECT_MS) return;
    uint32_t limit = (lr()->getDutyRxUs() + lr()->getDutySleepUs()) * 2 / 1000 + 10;
    for (uint32_t t0 = millis(); millis() - t0 < limit; ) {
      if (!lr()->dutyAsleep()) { _busyLowAt = millis(); return; }   // next window came: it is cycling
    }
    lr()->standby();              // NOP wake, then STBY_RC
    _stuckIrq = lr()->diagIrq();
    _stuckWakes++;
    _stuckAt = now;
    idle();
    startRecv();
    _busyLowAt = millis();
  }
public:
  uint32_t getStuckWakes() const { return _stuckWakes; }
  uint32_t getStuckIrq() const { return _stuckIrq; }
  uint32_t getStuckAt() const { return _stuckAt; }
#endif

public:
  CustomLR1110Wrapper(CustomLR1110& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

#ifdef LR11X0_DIAG
  // main-loop gaps (time between two passes) and transmissions (start → finished)
  uint32_t _diagLast = 0, _gapMax = 0, _gap200 = 0, _gap1s = 0, _txStart = 0, _txMax = 0, _txStarted = 0,
           _txDone = 0, _txFail = 0;
  uint32_t _gapAt[8] = {}, _gapLen[8] = {};
  uint8_t _gapHead = 0;
public:
  void printDiag(Print& out) {
    static const char* names[] = {"startRx", "readData", "startTx", "standby", "irqStatus", "rssiInst"};
    CustomLR1110* r = lr();
    out.printf("rxdiag @%lus: duty=%d active=%d busy=%d irq=%d wakes=%lu rx=%lu rxerr=%lu sent=%lu\r\n",
               millis() / 1000, getRxDutyCycle(), r->isDutyActive(), r->busyNow(), r->irqPending(),
               (unsigned long)r->getWakes(), (unsigned long)n_recv, (unsigned long)n_recv_errors, (unsigned long)n_sent);
    out.printf("  stuck asleep: %lu (last at %lus, irq 0x%08lx)  standby retries: %lu\r\n", (unsigned long)_stuckWakes,
               (unsigned long)(_stuckAt / 1000), (unsigned long)_stuckIrq, (unsigned long)r->getStandbyRetries());
    for (int i = 0; i < CustomLR1110::D_N; i++) {
      auto& d = r->diag[i];
      out.printf("  %-9s n=%lu err=%lu last=%d max=%lums avg=%luus\r\n", names[i], (unsigned long)d.calls,
                 (unsigned long)d.errs, d.lastErr, (unsigned long)(d.maxUs / 1000),
                 (unsigned long)(d.calls ? d.totalUs / d.calls : 0));
    }
    out.printf("  loop (active time between passes): max=%lums >200ms:%lu >1s:%lu | tx started=%lu done=%lu fail=%lu max=%lums\r\n",
               (unsigned long)_gapMax, (unsigned long)_gap200, (unsigned long)_gap1s, (unsigned long)_txStarted,
               (unsigned long)_txDone, (unsigned long)_txFail, (unsigned long)_txMax);
    out.print("  gaps:");
    for (int i = 0; i < 8; i++) {
      int k = (_gapHead + 7 - i) % 8;
      if (_gapLen[k]) out.printf(" %lus:%lums", (unsigned long)(_gapAt[k] / 1000), (unsigned long)_gapLen[k]);
    }
    out.printf("\r\n  events (%lu):", (unsigned long)r->diagEvts);
    for (int i = 0; i < 32; i++) {
      auto& e = r->diagRing[(r->diagHead + 31 - i) % 32];
      if (!e.ms) continue;
      out.printf(" [%lus %s %d %lums b%u]", (unsigned long)(e.ms / 1000), names[e.op], e.err,
                 (unsigned long)(e.us / 1000), e.busy);
      if (i % 4 == 3) out.print("\r\n   ");
    }
    out.print("\r\n");
    static const char* snrN[] = {"<-10", "-10", "-5", "0", "5", ">=10"};
    static const char* rssiN[] = {"<-115", "-115", "-105", "-95", "-85", ">=-75"};
    out.printf("  rx rejected: crc %lu, header %lu, other %lu\r\n", (unsigned long)r->rxErrCrc,
               (unsigned long)r->rxErrHdr, (unsigned long)r->rxErrOther);
    out.print("  snr dB  (ok/rejected):");
    for (int i = 0; i < CustomLR1110::RX_HIST; i++)
      out.printf(" %s:%lu/%lu", snrN[i], (unsigned long)r->rxSnrOk[i], (unsigned long)r->rxSnrErr[i]);
    out.print("\r\n  rssi dBm (ok/rejected):");
    for (int i = 0; i < CustomLR1110::RX_HIST; i++)
      out.printf(" %s:%lu/%lu", rssiN[i], (unsigned long)r->rxRssiOk[i], (unsigned long)r->rxRssiErr[i]);
    out.print("\r\n  last rejected [s irq rssi snr len hdr path]:");
    for (int i = 0; i < 16; i++) {
      auto& e = r->rxErrRing[(r->rxErrHead + 15 - i) % 16];
      if (!e.ms) continue;
      int a4 = abs(e.snr4);
      out.printf(" [%lus %lx %d %s%d.%02d %u %02x %02x]", (unsigned long)(e.ms / 1000), (unsigned long)e.irq, e.rssi,
                 e.snr4 < 0 ? "-" : "", a4 / 4, (a4 % 4) * 25, e.len, e.b0, e.b1);
      if (i % 4 == 3) out.print("\r\n   ");
    }
    out.print("\r\n");
  }
  void resetDiag() {
    CustomLR1110* r = lr();
    r->resetRxStats();
    memset(r->diag, 0, sizeof(r->diag)); memset(r->diagRing, 0, sizeof(r->diagRing)); r->diagEvts = 0;
    _gapMax = _gap200 = _gap1s = _txMax = _txStarted = _txDone = _txFail = 0;
    memset(_gapLen, 0, sizeof(_gapLen));
  }
#endif

#ifdef LR11X0_RX_DUTY_CYCLE
  void loop() override {
    checkStuckAsleep();
#if defined(LR11X0_DIAG) && defined(GR551X_PLATFORM)
    // active time between two passes = elapsed - CPU idle sleep (the CPU sleeps up to 1 s between passes)
    gr551x_power_stats_t ps;
    gr551x_power_stats_get(&ps);
    uint32_t now = millis();
    uint32_t busy = (uint32_t)((ps.total_ticks - ps.idle_ticks) * 1000 >> 15);
    if (_diagLast && busy >= _diagLast) {      // ("power reset" restarts the counters: new baseline)
      uint32_t gap = busy - _diagLast;
      if (gap > _gapMax) _gapMax = gap;
      if (gap > 200) { _gap200++; _gapAt[_gapHead] = now; _gapLen[_gapHead] = gap; _gapHead = (_gapHead + 1) % 8; }
      if (gap > 1000) _gap1s++;
    }
    _diagLast = busy ? busy : 1;
#endif
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
      lr()->setRxDutyCycle(_dutyWanted && !_dutyHeld);
      if (isInRecvMode()) startRecv();
    }
  }

  // RX duty cycle on/off, user setting (applied at the next startReceive(); a noise-floor round runs first when
  // the cycle starts)
  void setRxDutyCycle(bool en) {
    _dutyWanted = en;
    if (en) _floorKnown = false;
    _calibrating = false;
    lr()->setRxDutyCycle(en && !_dutyHeld);
    idle();
  }
  bool getRxDutyCycle() const { return _dutyWanted; }
  // hold the cycle off (continuous receive) while true, e.g. companion app connected
  void setRxDutyHeld(bool held) {
    if (held == _dutyHeld) return;
    _dutyHeld = held;
    applyDuty();
  }
  bool getRxDutyHeld() const { return _dutyHeld; }
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
    bool ok = RadioLibWrapper::startSendRaw(bytes, len);
#ifdef LR11X0_DIAG
    if (ok) { _txStarted++; _txStart = millis(); } else _txFail++;
#endif
    return ok;
  }

  // Before transmitting: asleep in its cycle (BUSY high) means nothing is being received, without SPI
  bool isReceiving() override {
    if (lr()->isDutyActive()) {
      if (!lr()->dutyAsleep() && lr()->dutyActivity()) return true;
      return isChannelActive();
    }
    return RadioLibWrapper::isReceiving();
  }
#endif

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
#ifdef LR11X0_RX_DUTY_CYCLE
    ((CustomLR1110 *)_radio)->leaveDuty();
#endif
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
    // called by recvRaw() right after readData(), before receive restarts: latch the packet RSSI/SNR now
    ((CustomLR1110 *)_radio)->latchPacketStatus();
    bool crc = true;
    ((CustomLR1110 *)_radio)->getLoRaRxHeaderInfo(NULL, &crc);
    return crc;
  }
  bool isReceivingPacket() override {
#ifdef LR11X0_RX_DUTY_CYCLE
    if (dutyQuiet()) return false;          // noise-floor sampling / AGC reset: no SPI during the duty cycle
#endif
    return ((CustomLR1110 *)_radio)->isReceiving();
  }
  float getCurrentRSSI() override {
#ifdef LR11X0_RX_DUTY_CYCLE
    if (dutyQuiet()) return _noise_floor;   // last measured floor (int.thresh compares against it)
#endif
    float rssi = -110;
    ((CustomLR1110 *)_radio)->getRssiInstDiag(&rssi);
    return rssi;
  }

  uint32_t getEstAirtimeFor(int len_bytes) override {
    uint32_t airtime = ((CustomLR1110 *)_radio)->getLoRaTimeOnAir(len_bytes) / 1000;   // no SPI (see CustomLR1110.h)
    return airtime < 200 ? 200 : airtime;   // at least 200 millis
  }

  void onSendFinished() override {
#ifdef LR11X0_DIAG
    _txDone++;
    if (_txStart && millis() - _txStart > _txMax) _txMax = millis() - _txStart;
    _txStart = 0;
#endif
    RadioLibWrapper::onSendFinished();
    _radio->setPreambleLength(preambleLengthForSF(getSpreadingFactor())); // overcomes weird issues with small and big pkts
  }

  float getLastRSSI() const override { return ((CustomLR1110 *)_radio)->getLastPacketRSSI(); }
  float getLastSNR() const override { return ((CustomLR1110 *)_radio)->getLastPacketSNR(); }

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

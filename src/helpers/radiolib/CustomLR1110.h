#pragma once

#include <RadioLib.h>
#include "MeshCore.h"

// LR11X0_RX_DUTY_CYCLE (defined, 0 or 1): build the RX duty cycle support, with this initial state (the board can
// switch it at run time). Not defined: the driver behaves exactly as upstream.
// LR11X0_DIAG: timing and error counters of every RadioLib call (debug builds, needs the duty cycle support).
#if defined(LR11X0_DIAG) && !defined(LR11X0_RX_DUTY_CYCLE)
  #error "LR11X0_DIAG needs LR11X0_RX_DUTY_CYCLE"
#endif

class CustomLR1110 : public LR1110 {
  uint32_t _preambleMillis = 66;
  uint32_t _maxPayloadMillis = 3934;
  uint32_t _activityAt = 0;
  bool _headerSeen = false;
  bool _rx_boosted = false;
  float _lastPktRssi = 0, _lastPktSnr = 0;
#ifdef LR11X0_RX_DUTY_CYCLE
  bool _rxDuty = LR11X0_RX_DUTY_CYCLE;   // receive in RX duty cycle (sleep between short listening windows)
  bool _dutyActive = false;              // the last startReceive() actually started a duty cycle
  uint32_t _dutyRxUs = 0, _dutySleepUs = 0;
  uint32_t _wakes = 0;                   // NOP wake-ups of a sleeping radio (diagnostics)
  uint32_t _standbyRetries = 0;          // STANDBY retried after a BUSY timeout (radio fell asleep meanwhile)
#endif

  static const uint32_t RX_FLAGS = RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED);

  public:
#ifdef LR11X0_DIAG
    // Diagnostics (debug build): duration and result of every RadioLib call the driver makes, plus a ring of
    // abnormal events (error, or call longer than 20 ms). RadioLib waits for BUSY low before each command, so a
    // call blocked by a sleeping radio shows up here as a long duration or a -705 (SPI command timeout).
    enum { D_RX, D_READ, D_TX, D_STBY, D_IRQ, D_RSSI, D_N };
    struct DiagOp { uint32_t calls, errs, maxUs, totalUs; int16_t lastErr; };
    struct DiagEvt { uint32_t ms; uint8_t op; int16_t err; uint32_t us; uint8_t busy; };
    DiagOp diag[D_N] = {};
    DiagEvt diagRing[32] = {};
    uint8_t diagHead = 0;
    uint32_t diagEvts = 0;
    void diagNote(uint8_t op, int16_t err, uint32_t us, bool busyBefore) {
      DiagOp& d = diag[op];
      d.calls++; d.totalUs += us;
      if (us > d.maxUs) d.maxUs = us;
      if (err != RADIOLIB_ERR_NONE) { d.errs++; d.lastErr = err; }
      if (err != RADIOLIB_ERR_NONE || us > 20000) {
        diagRing[diagHead] = {millis(), op, err, us, busyBefore};
        diagHead = (diagHead + 1) % 32;
        diagEvts++;
      }
    }
    bool busyNow() { return mod->hal->digitalRead(mod->getGpio()) == mod->hal->GpioLevelHigh; }
  #define LR_DIAG(op, expr) ({ bool _b = busyNow(); uint32_t _t = micros(); int16_t _e = (expr); \
                               diagNote(op, _e, micros() - _t, _b); _e; })
#else
  #define LR_DIAG(op, expr) (expr)
#endif
    CustomLR1110(Module *mod) : LR1110(mod) { }

    int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7,
                  uint8_t syncWord = RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, int8_t power = 10,
                  uint16_t preambleLength = 8, float tcxoVoltage = 1.6) {
      int16_t state = LR1110::begin(freq, bw, sf, cr, syncWord, power, preambleLength,
                                    tcxoVoltage);
      // RadioLib begin() defaults to LDO; use the LR1110 DC/DC regulator.
      if (state == RADIOLIB_ERR_NONE) state = setRegulatorDCDC();
      return state;
    }

    size_t getPacketLength(bool update) override {
      size_t len = LR1110::getPacketLength(update);
      if (len == 0 && getIrqStatus() & RADIOLIB_LR11X0_IRQ_HEADER_ERR) {
        // we've just received a corrupted packet
        // this may have triggered a bug causing subsequent packets to be shifted
        // call standby() to return radio to known-good state
        // recvRaw will call startReceive() to restart rx
        MESH_DEBUG_PRINTLN("LR1110: got header err, calling standby()");
        standby();
      }
      return len;
    }
    
    float getFreqMHz() const { return freqMHz; }

    int16_t setRxBoostedGainMode(bool en) {
      int16_t status = LR1110::setRxBoostedGainMode(en);
      if (status == RADIOLIB_ERR_NONE) _rx_boosted = en;
      return status;
    }

    bool getRxBoostedGainMode() const { return _rx_boosted; }

    // Time on air of a LoRa packet, from the configured modulation only (no SPI). RadioLib's getTimeOnAir() first
    // reads the packet type from the radio: with the radio asleep (BUSY high) that read times out after 1 s, and
    // getTimeOnAir() then returns an error code (-20) that reads as a duration of ~71 min. The Dispatcher took it as
    // the airtime of the next packet and postponed every transmission for hours (T1000-A, 2026-11-18).
    RadioLibTime_t getLoRaTimeOnAir(size_t len) { return getToA(len, RADIOLIB_MODEM_LORA); }

    // RSSI/SNR of the last received packet, latched right after reading it: receive restarts just after, and a later
    // read would go to SPI while the radio may sleep in its duty cycle (1 s timeout), or report another state.
    void latchPacketStatus() {
      float rssi = 0, snr = 0;
      if (getPacketStatusLoRa(&rssi, &snr, NULL) == RADIOLIB_ERR_NONE) { _lastPktRssi = rssi; _lastPktSnr = snr; }
    }
    float getLastPacketRSSI() const { return _lastPktRssi; }
    float getLastPacketSNR() const { return _lastPktSnr; }

#ifdef LR11X0_RX_DUTY_CYCLE
    // Every standby wakes the radio with a NOP first: in duty cycle it may be asleep (between windows, or after a
    // packet received), and RadioLib's commands wait for BUSY low before pulling NSS, so they would never wake it.
    int16_t standby() override {
      _dutyActive = false;
      int16_t st = LR_DIAG(D_STBY, LR1110::standby(RADIOLIB_LR11X0_STANDBY_RC, true));
      // A command landing just as the radio falls asleep between two windows can leave it asleep for good (BUSY high,
      // no RTC wake-up): on the T1000-A, 2026-11-18, STANDBY timed out after 1 s once in ~360 transmissions, with
      // BUSY low just before. The radio is then really asleep: a second NOP wakes it, retry once.
      if (st == RADIOLIB_ERR_SPI_CMD_TIMEOUT) {
        _standbyRetries++;
        st = LR_DIAG(D_STBY, LR1110::standby(RADIOLIB_LR11X0_STANDBY_RC, true));
      }
      return st;
    }
    uint32_t getStandbyRetries() const { return _standbyRetries; }
#endif

    int16_t startReceive() override {
#ifdef LR11X0_RX_DUTY_CYCLE
      // leave the cycle (or the sleep after a packet) before staging: the radio could sleep between commands
      if (_dutyActive || mod->hal->digitalRead(mod->getGpio()) == mod->hal->GpioLevelHigh) standby();
      if (_rxDuty && computeDutyCycle()) {
        // Duty cycle: the LR1110 sleeps between listening windows long enough to catch the senders' preamble
        // (RadioLib's calculation, with our own preamble length as the senders'). DIO1 mapping is unchanged.
        int16_t state = LR_DIAG(D_RX, startReceiveDutyCycle(_dutyRxUs, _dutySleepUs, RX_FLAGS, RADIOLIB_IRQ_RX_DEFAULT_MASK));
        _dutyActive = state == RADIOLIB_ERR_NONE;
        return state;
      }
      _dutyActive = false;
#endif
      // include the PREAMBLE_DETECTED irq bit in reported flags.
      return LR_DIAG(D_RX, LR1110::startReceive(RADIOLIB_LR11X0_RX_TIMEOUT_INF, RX_FLAGS, RADIOLIB_IRQ_RX_DEFAULT_MASK, 0));
    }

#ifdef LR11X0_DIAG
    // Received packets, good vs rejected (debug): RadioLib reports -7 (CRC_MISMATCH) both for a payload CRC error and
    // for a header error without a valid header (possibly a stale flag): the IRQ flags, read before readData() clears
    // them, tell which. SNR/RSSI histograms compare rejected packets with good ones (collisions vs weak signals).
    enum { RX_HIST = 6 };
    struct RxErr { uint32_t ms; uint32_t irq; int16_t rssi; int8_t snr4; uint8_t len, b0, b1; };
    uint32_t rxSnrOk[RX_HIST] = {}, rxSnrErr[RX_HIST] = {}, rxRssiOk[RX_HIST] = {}, rxRssiErr[RX_HIST] = {};
    uint32_t rxErrCrc = 0, rxErrHdr = 0, rxErrOther = 0;
    RxErr rxErrRing[16] = {};
    uint8_t rxErrHead = 0;
    static uint8_t snrBucket(float snr) {      // < -10, -10..-5, -5..0, 0..5, 5..10, >= 10 dB
      return snr < -10 ? 0 : snr < -5 ? 1 : snr < 0 ? 2 : snr < 5 ? 3 : snr < 10 ? 4 : 5;
    }
    static uint8_t rssiBucket(float rssi) {    // < -115, -115..-105, -105..-95, -95..-85, -85..-75, >= -75 dBm
      return rssi < -115 ? 0 : rssi < -105 ? 1 : rssi < -95 ? 2 : rssi < -85 ? 3 : rssi < -75 ? 4 : 5;
    }
    void resetRxStats() {
      memset(rxSnrOk, 0, sizeof(rxSnrOk)); memset(rxSnrErr, 0, sizeof(rxSnrErr));
      memset(rxRssiOk, 0, sizeof(rxRssiOk)); memset(rxRssiErr, 0, sizeof(rxRssiErr));
      memset(rxErrRing, 0, sizeof(rxErrRing)); rxErrCrc = rxErrHdr = rxErrOther = 0;
    }
    int16_t readData(uint8_t* data, size_t len) override {
      uint32_t irq = getIrqStatus();
      int16_t e = LR_DIAG(D_READ, LR1110::readData(data, len));
      float rssi = 0, snr = 0;
      getPacketStatusLoRa(&rssi, &snr, NULL);
      if (e == RADIOLIB_ERR_NONE) {
        rxSnrOk[snrBucket(snr)]++; rxRssiOk[rssiBucket(rssi)]++;
      } else {
        rxSnrErr[snrBucket(snr)]++; rxRssiErr[rssiBucket(rssi)]++;
        if (irq & RADIOLIB_LR11X0_IRQ_CRC_ERR) rxErrCrc++;
        else if (irq & RADIOLIB_LR11X0_IRQ_HEADER_ERR) rxErrHdr++;
        else rxErrOther++;
        rxErrRing[rxErrHead] = {millis(), irq, (int16_t)rssi, (int8_t)(snr * 4), (uint8_t)len,
                                len > 0 ? data[0] : (uint8_t)0, len > 1 ? data[1] : (uint8_t)0};
        rxErrHead = (rxErrHead + 1) % 16;
      }
      return e;
    }
    int16_t startTransmit(const uint8_t* data, size_t len, uint8_t addr = 0) override {
      return LR_DIAG(D_TX, LR1110::startTransmit(data, len, addr));
    }
    uint32_t diagIrq() {                     // getIrqStatus() timed (it returns the flags, not an error code)
      bool b = busyNow(); uint32_t t = micros();
      uint32_t irq = getIrqStatus();
      diagNote(D_IRQ, RADIOLIB_ERR_NONE, micros() - t, b);
      return irq;
    }
#else
    uint32_t diagIrq() { return getIrqStatus(); }
#endif
    int16_t getRssiInstDiag(float* rssi) { return LR_DIAG(D_RSSI, getRssiInst(rssi)); }

#ifdef LR11X0_RX_DUTY_CYCLE
    // RX duty cycle periods for the current modulation (PhysicalLayer::calculateRxDutyCycle, as
    // startReceiveDutyCycleAuto); false when the sleep would be too short to be worth it (e.g. 16-symbol preamble)
    bool computeDutyCycle() {
      DataRate_t dr;
      dr.lora.spreadingFactor = spreadingFactor;
      dr.lora.bandwidth = bandwidthKhz;
      dr.lora.codingRate = codingRate;
      uint32_t wake = 0, sleep = 0;
      if (calculateRxDutyCycle(0, preambleLengthLoRa, 0, &dr, &wake, &sleep) != RADIOLIB_ERR_NONE) return false;
      if (sleep < tcxoDelay + 1016) return false;
      // Once a preamble is detected, the LR1110 waits for the header at most 2 × rxPeriod + sleepPeriod (UM §7.2.6).
      // RadioLib only covers the preamble: the 4.25 sync + 8 header symbols that follow it are missing, so a
      // preamble detected early in its 32 symbols times out before its header (measured: 6 frames out of 20).
      // Cover preamble + header + 1 symbol of margin, counting from a detection 2 symbols into the preamble
      // (measured: ≈ 3.25), with the sleep RadioLib programs (minus TCXO start-up and 1 ms).
      uint32_t sym = ((uint32_t)(10 * 1000) << spreadingFactor) / (uint32_t)(10 * bandwidthKhz);
      uint32_t need = sym * preambleLengthLoRa + sym * 45 / 4 + sym;     // preamble + 12.25 - 2 + 1 symbols
      uint32_t programmedSleep = sleep - tcxoDelay - 1000;
      if (2 * wake + programmedSleep < need) wake = (need - programmedSleep + 1) / 2;
      _dutyRxUs = wake;
      _dutySleepUs = sleep;
      return true;
    }

    // Leave the duty cycle cleanly before transmitting or reconfiguring: RadioLib stages a transmission with
    // several commands, each waiting for a listening window, and the radio may sleep in between (the TX buffer
    // would not survive). standby() wakes it (NOP, no BUSY wait) then sets STBY_RC once it is ready.
    void leaveDuty() {
      if (_dutyActive) standby();
    }

    // In duty cycle the LR1110 goes back to sleep once a packet is received (BUSY high). RadioLib's commands wait
    // for BUSY low before pulling NSS, so they would never wake it: each one times out (1 s) and fails, the packet
    // is never read, receive is never restarted and transmissions fail. Wake it with a NOP first (standby()).
    bool wakeIfAsleep() {
      if (mod->hal->digitalRead(mod->getGpio()) != mod->hal->GpioLevelHigh) return false;
      standby();
      _wakes++;
      return true;
    }
    // interrupt line asserted (packet received, TX done...)
    bool irqPending() { return mod->hal->digitalRead(mod->getIrq()) == mod->hal->GpioLevelHigh; }
    uint32_t getWakes() const { return _wakes; }

    void setRxDutyCycle(bool en) { _rxDuty = en; }   // applied at the next startReceive()
    bool getRxDutyCycle() const { return _rxDuty; }
    bool isDutyActive() const { return _dutyActive; }
    // listening window and sleep (µs, sleep including the wake-up transition)
    uint32_t getDutyRxUs() const { return _dutyRxUs; }
    uint32_t getDutySleepUs() const { return _dutySleepUs; }

    // Duty cycle: the LR1110 keeps BUSY high while asleep between windows. Then nothing is being received, and
    // any SPI access would wake it up and end the cycle: check this pin before touching the radio.
    bool dutyAsleep() {
      return _dutyActive && mod->hal->digitalRead(mod->getGpio()) == mod->hal->GpioLevelHigh;
    }

    // Duty cycle, radio awake (BUSY low): is a packet coming in? A valid header without RX done means yes. A
    // preamble alone counts once and is cleared: it stays set after a header timeout, which would block TX.
    bool dutyActivity() {
      uint32_t irq = diagIrq();
      if (irq & (RADIOLIB_LR11X0_IRQ_RX_DONE | RADIOLIB_LR11X0_IRQ_HEADER_ERR)) return false;
      if (irq & RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID) return true;
      if (irq & RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED) {
        clearIrqState(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED);
        return true;
      }
      return false;
    }
#endif

    bool isReceiving() {
      uint32_t irq = diagIrq();
      bool preamble = irq & RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED;      // bit 4
      bool header   = irq & RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID; // bit 5
      bool hdrErr   = irq & RADIOLIB_LR11X0_IRQ_HEADER_ERR;             // bit 6
      uint32_t now  = millis();
      if (hdrErr) {
        clearIrqState(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED | RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID | RADIOLIB_LR11X0_IRQ_HEADER_ERR);
        _activityAt = 0;
        _headerSeen = false;
        return false;
      }
      if (!header && _headerSeen) {
        // something cleared the header flag, reset our state.
        _activityAt = 0; _headerSeen = false;
        return false;
      }
      if (header) {
        if (!_headerSeen) { _headerSeen = true; _activityAt = now; };
        if (now - _activityAt > _maxPayloadMillis) {
          MESH_DEBUG_PRINTLN("Clearing header IRQ after %ums", _maxPayloadMillis);
          clearIrqState(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED | RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID | RADIOLIB_LR11X0_IRQ_HEADER_ERR);
          _activityAt = 0; _headerSeen = false;
          return false;
        }
        return true;
      }
      if (preamble) {
        if (_activityAt == 0) _activityAt = now;
        if (now - _activityAt > _preambleMillis) {
          clearIrqState(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED);
          _activityAt = 0;
          MESH_DEBUG_PRINTLN("Clearing preamble IRQ after %ums", _preambleMillis);

          return false;
        }
        return true;
      }
      _activityAt = 0; _headerSeen = false;
      return false;
    }
    
    void setPreambleMillis(uint32_t preambleMillis) {
      _preambleMillis = preambleMillis;
      MESH_DEBUG_PRINTLN("Set _preambleMillis=%u", _preambleMillis);
    }
    void setMaxPayloadMillis(uint32_t payloadMillis) {
      _maxPayloadMillis = payloadMillis;
      MESH_DEBUG_PRINTLN("Set _maxPayloadMillis=%u", _maxPayloadMillis);
    }

    uint8_t getSpreadingFactor() const { return spreadingFactor; }
};

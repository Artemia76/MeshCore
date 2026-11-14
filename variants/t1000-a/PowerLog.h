#pragma once

#include <Arduino.h>
#include <MeshCore.h>

// Software power accounting for the T1000-A (no current meter needed): time spent in each state (CPU idle,
// BLE connected / advertising, LoRa TX, GNSS on), summed over 10 min intervals into a RAM ring buffer (24 h).
// Read through the BLE console: "power", "power log", "power reset", "chg". Currents per state come from
// the datasheets, so the budget is computed off-device from these times.
class T1000aPowerLog {
public:
  static const uint16_t SAMPLES = 144;          // 24 h
  static const uint32_t PERIOD_MS = 600000UL;   // 10 min

  // flags: state when the battery was read
  enum {
    F_GPS_ON    = 0x01,   // GPS_EN high
    F_BLE_CONN  = 0x02,   // a BLE connection was open
    F_TX_WAITED = 0x04,   // the reading was postponed until the end of a LoRa transmission
    F_AON4      = 0x08,   // charge-state candidates (raw levels, pull-up)
    F_AON5      = 0x10,
    F_GPIO14    = 0x20,   // battery/sensor divider enabled
    F_BUZZER_EN = 0x40,
  };

  struct Sample {
    uint32_t t_s;         // end of the interval, seconds since reset
    uint32_t tx_ms;       // LoRa transmit time in the interval
    uint16_t dt_s;        // length of the interval
    uint16_t batt_mv;     // average of 8 readings
    uint16_t idle_pm;     // CPU idle, per mille of the interval
    uint16_t wakes;       // wake-up interrupts in the interval
    uint16_t conn_s;      // BLE connection open
    uint16_t adv_s;       // BLE advertising
    uint16_t gps_s;       // GNSS powered
    uint16_t rx_pkts, tx_pkts;
    uint16_t msio0_mv;    // charge-state candidate (ADC)
    uint8_t flags;
  } __attribute__((packed));

  void begin(mesh::MainBoard* board);
  void loop();
  void txStart();
  void txEnd();
  void reset();

  void printSummary(Print& out);
  void printLog(Print& out);
  void printCharge(Print& out);

private:
  struct Totals {         // cumulative since reset
    uint64_t total, idle, conn, adv;   // 32 kHz ticks
    uint32_t wakes, sleeps, refused_event, refused_awake, loops;
    uint32_t gps_ms, tx_ms, rx_pkts, tx_pkts;
  };

  mesh::MainBoard* _board = nullptr;
  Sample _log[SAMPLES];
  uint16_t _head = 0, _count = 0;

  uint32_t _last_ms = 0, _sample_due_ms = 0;
  bool _gps_on = false;
  uint32_t _gps_ms = 0, _tx_ms = 0, _tx_start = 0;
  bool _tx_active = false, _tx_waited = false;
  uint32_t _rx_base = 0, _tx_base = 0;
  Totals _prev;           // totals at the last sample

  void totals(Totals& t);
  uint16_t battMilliVolts();
  uint8_t probeFlags();
  void takeSample();
};

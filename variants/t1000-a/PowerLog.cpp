#include "PowerLog.h"

void t1000aRadioCounts(uint32_t& rx, uint32_t& tx);   // target.cpp

#define PIN_CHG_AON4  GR551X_PIN_AON(4)
#define PIN_CHG_AON5  GR551X_PIN_AON(5)
#define PIN_CHG_MSIO0 GR551X_PIN_MSIO(0)

static uint16_t sat16(uint64_t v) { return v > 0xFFFF ? 0xFFFF : (uint16_t)v; }
static uint32_t ticksToSec(uint64_t t) { return (uint32_t)(t >> 15); }

// "12.3 %" from a ratio, without floating point printf
static void printPct(Print& out, uint64_t part, uint64_t whole) {
  uint32_t pm = whole ? (uint32_t)((part * 1000 + whole / 2) / whole) : 0;
  out.printf("%lu.%lu %%", (unsigned long)(pm / 10), (unsigned long)(pm % 10));
}

void T1000aPowerLog::begin(mesh::MainBoard* board) {
  _board = board;
  // Charge-state candidates (docs/materiel-t1000a.md): read raw, the right one is identified on the board
  pinMode(PIN_CHG_AON4, INPUT_PULLUP);
  pinMode(PIN_CHG_AON5, INPUT_PULLUP);
  reset();
}

void T1000aPowerLog::reset() {
  gr551x_power_stats_reset();
  t1000aRadioCounts(_rx_base, _tx_base);
  _gps_ms = _tx_ms = 0;
  _tx_waited = false;
  if (_tx_active) _tx_start = millis();
  _head = _count = 0;
  _last_ms = millis();
  _sample_due_ms = _last_ms + PERIOD_MS;
  _gps_on = digitalRead(GPS_EN) == HIGH;
  totals(_prev);
}

void T1000aPowerLog::totals(Totals& t) {
  gr551x_power_stats_t s;
  gr551x_power_stats_get(&s);
  t.total = s.total_ticks;
  t.idle = s.idle_ticks;
  t.conn = s.ble_conn_ticks;
  t.adv = s.ble_adv_ticks;
  t.wakes = s.wake_events;
  t.sleeps = s.idle_entries;
  t.refused_event = s.idle_refused_event;
  t.refused_awake = s.idle_refused_awake;
  t.loops = s.loops;
  t.gps_ms = _gps_ms;
  t.tx_ms = _tx_ms + (_tx_active ? millis() - _tx_start : 0);
  uint32_t rx, tx;
  t1000aRadioCounts(rx, tx);
  t.rx_pkts = rx - _rx_base;
  t.tx_pkts = tx - _tx_base;
}

void T1000aPowerLog::txStart() {
  _tx_start = millis();
  _tx_active = true;
}

void T1000aPowerLog::txEnd() {
  if (!_tx_active) return;
  _tx_ms += millis() - _tx_start;
  _tx_active = false;
}

uint16_t T1000aPowerLog::battMilliVolts() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += _board->getBattMilliVolts();
  return (uint16_t)(sum / 8);
}

uint8_t T1000aPowerLog::probeFlags() {
  gr551x_power_stats_t s;
  gr551x_power_stats_get(&s);
  uint8_t f = 0;
  if (digitalRead(GPS_EN) == HIGH) f |= F_GPS_ON;
  if (s.ble_connections) f |= F_BLE_CONN;
  if (digitalRead(PIN_CHG_AON4) == HIGH) f |= F_AON4;
  if (digitalRead(PIN_CHG_AON5) == HIGH) f |= F_AON5;
  if (digitalRead(PIN_LORA_RF_EN) == HIGH) f |= F_GPIO14;
#ifdef PIN_BUZZER_EN
  if (digitalRead(PIN_BUZZER_EN) == HIGH) f |= F_BUZZER_EN;
#endif
  return f;
}

void T1000aPowerLog::loop() {
  // GNSS time: the state seen at the previous pass held until now (it only changes from the main loop)
  uint32_t now = millis();
  if (_gps_on) _gps_ms += now - _last_ms;
  _last_ms = now;
  _gps_on = digitalRead(GPS_EN) == HIGH;

  if ((int32_t)(now - _sample_due_ms) < 0) return;
  if (_tx_active) { _tx_waited = true; return; }   // voltage sags while transmitting: read once it is over
  takeSample();
  _sample_due_ms += PERIOD_MS;
  if ((int32_t)(now - _sample_due_ms) >= 0) _sample_due_ms = now + PERIOD_MS;   // after a long stall
}

void T1000aPowerLog::takeSample() {
  Totals t;
  totals(t);
  uint64_t dt = t.total - _prev.total;

  Sample& s = _log[_head];
  s.t_s = ticksToSec(t.total);
  s.dt_s = sat16(ticksToSec(dt));
  s.flags = probeFlags() | (_tx_waited ? F_TX_WAITED : 0);
  s.batt_mv = battMilliVolts();
  int mv = analogReadMilliVolts(PIN_CHG_MSIO0);
  s.msio0_mv = mv > 0 ? (uint16_t)mv : 0;
  s.idle_pm = dt ? (uint16_t)((t.idle - _prev.idle) * 1000 / dt) : 0;
  s.wakes = sat16(t.wakes - _prev.wakes);
  s.conn_s = sat16(ticksToSec(t.conn - _prev.conn));
  s.adv_s = sat16(ticksToSec(t.adv - _prev.adv));
  s.gps_s = sat16((t.gps_ms - _prev.gps_ms) / 1000);
  s.tx_ms = t.tx_ms - _prev.tx_ms;
  s.rx_pkts = sat16(t.rx_pkts - _prev.rx_pkts);
  s.tx_pkts = sat16(t.tx_pkts - _prev.tx_pkts);

  _head = (_head + 1) % SAMPLES;
  if (_count < SAMPLES) _count++;
  _tx_waited = false;
  _prev = t;
}

void T1000aPowerLog::printSummary(Print& out) {
  Totals t;
  totals(t);
  uint32_t secs = ticksToSec(t.total);
  gr551x_power_stats_t s;
  gr551x_power_stats_get(&s);

  out.printf("power: %lu s since reset (%luh%02lum)\r\n", (unsigned long)secs, (unsigned long)(secs / 3600),
             (unsigned long)(secs / 60 % 60));
  out.print("cpu: idle "); printPct(out, t.idle, t.total);
  out.printf("  sleeps %lu  refused %lu event / %lu awake  wakes %lu  loops %lu\r\n", (unsigned long)t.sleeps,
             (unsigned long)t.refused_event, (unsigned long)t.refused_awake, (unsigned long)t.wakes,
             (unsigned long)t.loops);
  out.print("ble: connected "); printPct(out, t.conn, t.total);
  out.print("  advertising "); printPct(out, t.adv, t.total);
  out.printf("  (open now: %u)\r\n", s.ble_connections);
  out.printf("lora: tx %lu ms (", (unsigned long)t.tx_ms); printPct(out, (uint64_t)t.tx_ms << 15, t.total * 1000);
  out.printf(")  pkts rx %lu tx %lu\r\n", (unsigned long)t.rx_pkts, (unsigned long)t.tx_pkts);
  out.printf("gps: on %lu s (", (unsigned long)(t.gps_ms / 1000)); printPct(out, (uint64_t)t.gps_ms << 15, t.total * 1000);
  out.printf(")  now %s\r\n", digitalRead(GPS_EN) == HIGH ? "on" : "off");
  out.printf("bat: %u mV  gpio14=%d\r\n", battMilliVolts(), digitalRead(PIN_LORA_RF_EN) == HIGH);
  out.printf("log: %u/%u samples, every %lu s\r\n", _count, SAMPLES, (unsigned long)(PERIOD_MS / 1000));
}

void T1000aPowerLog::printLog(Print& out) {
  out.print("# t_s,dt_s,bat_mv,idle_pm,wakes,conn_s,adv_s,gps_s,tx_ms,rx,tx,flags,msio0_mv\r\n");
  uint16_t first = (_head + SAMPLES - _count) % SAMPLES;
  for (uint16_t i = 0; i < _count; i++) {
    const Sample& s = _log[(first + i) % SAMPLES];
    out.printf("%lu,%u,%u,%u,%u,%u,%u,%u,%lu,%u,%u,0x%02x,%u\r\n", (unsigned long)s.t_s, s.dt_s, s.batt_mv, s.idle_pm,
               s.wakes, s.conn_s, s.adv_s, s.gps_s, (unsigned long)s.tx_ms, s.rx_pkts, s.tx_pkts, s.flags, s.msio0_mv);
  }
  out.printf("# %u samples\r\n", _count);
}

void T1000aPowerLog::printCharge(Print& out) {
  int mv = analogReadMilliVolts(PIN_CHG_MSIO0);
  out.printf("chg: aon4=%d aon5=%d msio0=%d mV bat=%u mV\r\n", digitalRead(PIN_CHG_AON4) == HIGH,
             digitalRead(PIN_CHG_AON5) == HIGH, mv, battMilliVolts());
}

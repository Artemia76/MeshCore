#include "T1000aBoard.h"
#include <gr551x_ble.h>
#include <helpers/ui/ChargeLed.h>

void t1000aRadioSleep();   // target.cpp
void t1000aRxDutyLoad();

#ifdef T1000A_BLE_CONSOLE
  #include <app_pwm.h>
  #include <BleConsole.h>
  #include <helpers/gr551x/SerialBLEInterface.h>

static T1000aBoard* s_board;
void t1000aConsoleGps(Print& out);   // target.cpp
void t1000aRxDutySet(bool on, bool save);
void t1000aRxDutyInfo(Print& out);
void t1000aRadioDiag(Print& out, bool reset);   // target.cpp
void companionLoopDiag(Print& out, bool reset);  // examples/companion_radio/main.cpp (COMPANION_LOOP_DIAG)
extern "C" uint64_t gr551x_ticks32k(void);       // platform core (32 kHz AON time base), for "sched cpu"
void t1000aSha256(uint8_t* hash, const uint8_t* msg, int len);   // target.cpp (MeshCore Utils), for "sched cpu"
void t1000aLrProbe(Print& out, int argc, char** argv);   // LrProbe.cpp

#include <diag.h>
#include <stdarg.h>
static void diagOut(const char* fmt, ...) {
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  BleConsole.print(b);
}

static bool consoleCommand(int argc, char** argv) {
  if (!strcmp(argv[0], "hang")) {    // hang : blocks the main loop (IRQs on) to test the watchdog capture
    BleConsole.print("hang: main loop stuck on purpose, watchdog reboot in ~8 s\r\n");
    delay(300);
    volatile uint32_t spin = 0;
    while (true) spin++;
  }
  if (!strcmp(argv[0], "diag")) {    // diag [clear] : cause of the last reboot, last fault, watchdog capture
    diag_report(diagOut);
    if (argc > 1 && !strcmp(argv[1], "clear")) { diag_begin(); BleConsole.print("diag: cleared\r\n"); }
    return true;
  }
  if (!strcmp(argv[0], "bat")) {
    int raw = analogReadMilliVolts(PIN_BATTERY);
    BleConsole.printf("battery: %u mV (MSIO4 %d mV)\r\n", s_board->getBattMilliVolts(), raw);
    return true;
  }
  if (!strcmp(argv[0], "gps")) {
    t1000aConsoleGps(BleConsole);
    return true;
  }
  if (!strcmp(argv[0], "beep")) {   // beep [Hz] : 500 ms tone, ignores the "buzzer off" setting
    unsigned freq = argc > 1 ? (unsigned)atoi(argv[1]) : 2000;
    digitalWrite(PIN_BUZZER_EN, HIGH);
    tone(PIN_BUZZER, freq, 500);
    BleConsole.printf("beep %u Hz\r\n", freq);
    return true;
  }
  if (!strcmp(argv[0], "power")) {   // power [log|reset] : time spent in each state (power budget)
    if (argc > 1 && !strcmp(argv[1], "log")) s_board->powerLog.printLog(BleConsole);
    else if (argc > 1 && !strcmp(argv[1], "reset")) { s_board->powerLog.reset(); BleConsole.print("power: reset\r\n"); }
    else s_board->powerLog.printSummary(BleConsole);
    return true;
  }
  if (!strcmp(argv[0], "led")) {     // led [0..255] : analogWrite on the status LED, then PWM1 / AON3 state
    if (argc > 1) analogWrite(PIN_LED, atoi(argv[1]));
    pwm_handle_t* h = app_pwm_get_handle(APP_PWM_ID_1);
    BleConsole.printf("pwm1: state=%d mode=%08lx upd=%08lx prd=%lu a0=%lu a1=%lu aq=%08lx\r\n", h ? (int)h->state : -1,
                      PWM1->MODE, PWM1->UPDATE, PWM1->PRD, PWM1->CMPA0, PWM1->CMPA1, PWM1->AQCTRL);
    BleConsole.printf("aon: mux=%08lx pad1=%08lx\r\n", MCU_SUB->AON_PAD_MUX_CTL, AON->AON_PAD_CTL1);
    return true;
  }
  if (!strcmp(argv[0], "rxdiag")) {  // rxdiag [reset] : radio driver diagnostics (RadioLib calls, loop gaps, TX)
    t1000aRadioDiag(BleConsole, argc > 1 && !strcmp(argv[1], "reset"));
    return true;
  }
  if (!strcmp(argv[0], "rxduty")) {  // rxduty [on|off] : LoRa receive in duty cycle (saved)
    if (argc > 1 && (!strcmp(argv[1], "on") || !strcmp(argv[1], "off"))) t1000aRxDutySet(!strcmp(argv[1], "on"), true);
    t1000aRxDutyInfo(BleConsole);
    return true;
  }
  if (!strcmp(argv[0], "chg")) {     // charge-state candidates, to compare with the cable plugged / unplugged
    s_board->powerLog.printCharge(BleConsole);
    return true;
  }
  if (!strcmp(argv[0], "lr")) {      // lr <test> : LR1110 probe (raw SPI), then reboot
    t1000aLrProbe(BleConsole, argc, argv);
    return true;
  }
#ifdef GR551X_SCHED_STATS
  if (!strcmp(argv[0], "sched")) {   // sched [reset | awake <s> | cpu] : pwr_mgmt_schedule() and loop() durations (twin model)
    if (argc > 1 && !strcmp(argv[1], "reset")) { gr551x_sched_stats_reset(); BleConsole.print("sched: reset\r\n"); }
    if (argc > 2 && !strcmp(argv[1], "awake")) {   // force the "no sleep" path, as when hasPendingWork() is stuck
      gr551x_sched_stats_reset();
      gr551x_stay_awake((uint32_t)atoi(argv[2]) * 1000);
      BleConsole.printf("sched: reset, CPU kept awake for %d s\r\n", atoi(argv[2]));
      return true;
    }
    if (argc > 1 && !strcmp(argv[1], "cpu")) {     // core clock and cost of code executed from Flash / RAM
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
      uint64_t t0 = gr551x_ticks32k();
      while (gr551x_ticks32k() == t0) {}             // align on a tick edge
      t0 = gr551x_ticks32k();
      uint32_t c0 = DWT->CYCCNT;
      while (gr551x_ticks32k() - t0 < 3277) {}       // ~100 ms
      uint32_t c1 = DWT->CYCCNT;
      uint64_t t1 = gr551x_ticks32k();
      unsigned long hz = (unsigned long)((uint64_t)(c1 - c0) * 32768 / (t1 - t0));
      // tight loop: 2 instructions per pass (subs + bne), executed from Flash (XIP cache)
      uint32_t n = 200000, ca = DWT->CYCCNT;
      __asm volatile("1: subs %0, %0, #1\n bne 1b" : "+r"(n) :: "cc");
      uint32_t loop_cyc = DWT->CYCCNT - ca;
      // cost of the time base and of AON registers, then a representative computation (SHA-256 of 1 KB);
      // the twin runs the same code: its instruction count against these cycles gives the CPI to model
      static volatile uint32_t sink;
      static uint8_t buf[1024], hash[32];
      for (int i = 0; i < (int)sizeof(buf); i++) buf[i] = (uint8_t)(i * 7 + sink);
      __disable_irq();
      ca = DWT->CYCCNT;
      for (int i = 0; i < 100; i++) sink += (uint32_t)gr551x_ticks32k();
      uint32_t t32 = DWT->CYCCNT - ca;
      ca = DWT->CYCCNT;
      for (int i = 0; i < 100; i++) sink += millis();
      uint32_t tms = DWT->CYCCNT - ca;
      ca = DWT->CYCCNT;
      for (int i = 0; i < 100; i++) sink += AON->TIMER_VAL;
      uint32_t traw = DWT->CYCCNT - ca;
      ca = DWT->CYCCNT;
      for (int i = 0; i < 100; i++) AON->AON_PAD_CTL1 = AON->AON_PAD_CTL1;
      uint32_t trw = DWT->CYCCNT - ca;
      ca = DWT->CYCCNT;
      t1000aSha256(hash, buf, sizeof(buf));
      uint32_t tsha = DWT->CYCCNT - ca;
      __enable_irq();
      sink += hash[0];
      BleConsole.printf("cpu: core %lu Hz; loop 2 insn x 200000 = %lu cycles\r\n", hz, (unsigned long)loop_cyc);
      BleConsole.printf("cpu: per call (cycles): ticks32k %lu, millis %lu, AON TIMER_VAL read %lu, AON_PAD_CTL1 rd+wr %lu;"
                        " sha256 1 KB %lu\r\n", (unsigned long)(t32 / 100), (unsigned long)(tms / 100),
                        (unsigned long)(traw / 100), (unsigned long)(trw / 100), (unsigned long)tsha);
      return true;
    }
    gr551x_sched_stats_t st;
    gr551x_sched_stats_get(&st);
    const char* names[3] = { "active", "idle", "loop" };
    const gr551x_sched_bucket_t* b[3] = { &st.active, &st.idle, &st.loop };
    for (int i = 0; i < 3; i++) {
      unsigned long avg_us = b[i]->n ? (unsigned long)(b[i]->ticks * 1000000ULL / 32768 / b[i]->n) : 0;
      BleConsole.printf("%-6s n=%lu ev=%lu avg=%luus max=%lums | <1ms %lu <10ms %lu <100ms %lu <1s %lu >=1s %lu\r\n",
                        names[i], (unsigned long)b[i]->n, (unsigned long)b[i]->with_event, avg_us,
                        (unsigned long)(b[i]->max_ticks * 1000UL / 32768), (unsigned long)b[i]->hist[0],
                        (unsigned long)b[i]->hist[1], (unsigned long)b[i]->hist[2], (unsigned long)b[i]->hist[3],
                        (unsigned long)b[i]->hist[4]);
    }
    return true;
  }
#endif
  if (!strcmp(argv[0], "pending")) { // pending [reset] : what keeps the CPU awake, slowest parts of loop()
    companionLoopDiag(BleConsole, argc > 1 && !strcmp(argv[1], "reset"));
    return true;
  }
  if (!strcmp(argv[0], "blelog")) {
    serialBLEDumpLog(BleConsole);
    return true;
  }
  return false;
}
#endif

void T1000aBoard::begin() {
  GR551xBoard::begin();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUTTON1, INPUT);
  pinMode(EXT_CHRG_DETECT, INPUT_PULLUP);   // charger status: open drain, low while charging

  // GPIO14: enables the battery/sensor measurement divider in Seeed's firmware (kept high for now)
  pinMode(PIN_LORA_RF_EN, OUTPUT);
  digitalWrite(PIN_LORA_RF_EN, HIGH);

  // GNSS (AG3335) off until the sensor manager starts it; RESET is active low on the T1000-A
  pinMode(GPS_EN, OUTPUT);        digitalWrite(GPS_EN, LOW);
  pinMode(GPS_RESET, OUTPUT);     digitalWrite(GPS_RESET, HIGH);
  pinMode(GPS_VRTC_EN, OUTPUT);   digitalWrite(GPS_VRTC_EN, HIGH);
  pinMode(GPS_SLEEP_INT, OUTPUT); digitalWrite(GPS_SLEEP_INT, HIGH);
  pinMode(GPS_RTC_INT, OUTPUT);   digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, INPUT_PULLUP);

  // Woken from a real power off: stays off unless the button is still held (shows the charge meanwhile)
  gr551x_off_info_t off;
  bool was_off = gr551x_power_off_state(&off);
  uint16_t wake_mv = 0;
  if (was_off) {
    offBoot(off);
    wake_mv = getBattMilliVolts();
  }

#ifdef T1000A_BLE_CONSOLE
  // Development console (Goodix UART service): "dfu" restarts into the BLE bootloader, Serial output
  // (MESH_DEBUG) is mirrored there. Not advertised: the companion service owns advertising.
  BleConsole.begin("T1000-A", "MeshCore", false);
  s_board = this;
  BleConsole.onCommand(consoleCommand);
  powerLog.begin(this);
  if (was_off) powerLog.setLastOff(off.batt_mv, wake_mv, off.wakes);
#endif
}

// Button held for `ms` from now (polled; the core keeps the watchdog and the 6 s DFU hold running)
bool T1000aBoard::holdButton(uint32_t ms) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_BUTTON1) == HIGH) {
    if (millis() - t0 >= ms) return true;
    delay(10);
  }
  return false;
}

// After a wake from power off, before MeshCore starts. Waking from ultra deep sleep goes through the ROM and the
// bootloader (about 2-3 s), which filters accidental presses: the box powers on if the button is still held now,
// so a press of about 3 s in all. Shorter presses put it back to sleep. With the charging cable in, the LED shows
// the charge (breathing / steady) until unplugged, or until the button is held 2 s to power on.
void T1000aBoard::offBoot(const gr551x_off_info_t& off) {
  bool on = false;
  if (cablePresent()) {
    uint32_t held = 0;
    while (cablePresent()) {
      analogWrite(PIN_LED, chargeLedLevel(millis(), digitalRead(EXT_CHRG_DETECT) == LOW));
      if (digitalRead(PIN_BUTTON1) == HIGH) {
        if (!held) held = millis();
        else if (millis() - held >= 2000) { on = true; break; }
      } else {
        held = 0;
      }
      delay(10);
    }
    digitalWrite(PIN_LED, LOW);   // back to GPIO
  } else {
    on = holdButton(300);
  }
  if (!on) sleepNow(off.batt_mv);

  for (int i = 0; i < 2; i++) {   // powering on
    digitalWrite(PIN_LED, HIGH); delay(150);
    digitalWrite(PIN_LED, LOW);  delay(150);
  }
  gr551x_power_off_clear();
}

// Charging cable plugged in: MSIO0 voltage, or the charger status (AON4 low) that may have woken us
bool T1000aBoard::cablePresent() {
  return isExternalPowered() || digitalRead(EXT_CHRG_DETECT) == LOW;
}

// Ultra deep sleep, woken by the button (AON0 high) or by charging starting (charger status AON4 low). The MSIO
// comparator on the cable voltage does not wake the GR5515 from this mode (tried on the board).
void T1000aBoard::sleepNow(uint16_t batt_mv) {
  digitalWrite(GPS_EN, LOW);
  digitalWrite(GPS_VRTC_EN, LOW);
  digitalWrite(PIN_LORA_RF_EN, LOW);
#ifdef PIN_BUZZER_EN
  digitalWrite(PIN_BUZZER_EN, LOW);
#endif
  digitalWrite(PIN_LED, LOW);
  while (digitalRead(PIN_BUTTON1) == HIGH) delay(10);   // a held button would wake us at once
  gr551x_wake_cfg_t cfg = {1u << 0, 1u << (EXT_CHRG_DETECT - 0x20), batt_mv};
  gr551x_power_off(&cfg);
}

bool T1000aBoard::isExternalPowered() {
  uint32_t now = millis();
  if (!_ext_power_valid || now - _ext_power_at >= 1000) {
    _ext_power = analogReadMilliVolts(PIN_EXT_POWER) > 1000;
    _ext_power_at = now;
    _ext_power_valid = true;
  }
  return _ext_power;
}

void T1000aBoard::loop() {
  static bool rxduty_loaded;
  if (!rxduty_loaded) { rxduty_loaded = true; t1000aRxDutyLoad(); }   // InternalFS is up by now
#ifdef T1000A_BLE_CONSOLE
  BleConsole.poll();
  powerLog.loop();
#endif
  GR551xBoard::loop();
}

// Long press (3 s, UITask) -> real power off. Holding on until 6 s restarts into the BLE DFU bootloader instead
// (handled by the platform variant). Every part goes to its lowest state, then the GR5515 enters ultra deep sleep;
// power on = button held 2 s (offBoot).
void T1000aBoard::powerOff() {
  uint16_t mv = getBattMilliVolts();   // before the measurement divider is cut
  digitalWrite(GPS_EN, LOW);
  t1000aRadioSleep();                  // LR1110 cold sleep; GNSS (with its backup supply), divider, buzzer: sleepNow()
  gr551x_ble_stop();

  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON1) == HIGH) delay(10);   // released before 6 s: really go off
  digitalWrite(PIN_LED, LOW);
  sleepNow(mv);
}

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
void t1000aLrProbe(Print& out, int argc, char** argv);   // LrProbe.cpp

static bool consoleCommand(int argc, char** argv) {
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
  if (!strcmp(argv[0], "rxdiag")) {  // rxdiag [reset] : radio driver diagnostics (rx duty cycle trial)
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

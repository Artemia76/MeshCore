#include "T1000aBoard.h"
#include <gr551x_ble.h>

#ifdef T1000A_BLE_CONSOLE
  #include <BleConsole.h>
  #include <helpers/gr551x/SerialBLEInterface.h>

static T1000aBoard* s_board;
void t1000aConsoleGps(Print& out);   // target.cpp

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
  if (!strcmp(argv[0], "chg")) {     // charge-state candidates, to compare with the cable plugged / unplugged
    s_board->powerLog.printCharge(BleConsole);
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

#ifdef T1000A_BLE_CONSOLE
  // Development console (Goodix UART service): "dfu" restarts into the BLE bootloader, Serial output
  // (MESH_DEBUG) is mirrored there. Not advertised: the companion service owns advertising.
  BleConsole.begin("T1000-A", "MeshCore", false);
  s_board = this;
  BleConsole.onCommand(consoleCommand);
  powerLog.begin(this);
#endif
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
#ifdef T1000A_BLE_CONSOLE
  BleConsole.poll();
  powerLog.loop();
#endif
  GR551xBoard::loop();
}

// Long press (3 s, UITask) -> "off". Holding on until 6 s restarts into the BLE DFU bootloader instead
// (handled by the platform variant). Soft off for now: GNSS and buzzer off, LR1110 held in reset, BLE stopped,
// the CPU waits for the next button press and then reboots.
void T1000aBoard::powerOff() {
  digitalWrite(GPS_EN, LOW);
  digitalWrite(P_LORA_RESET, LOW);
#ifdef PIN_BUZZER_EN
  digitalWrite(PIN_BUZZER_EN, LOW);
#endif
  gr551x_ble_stop();

  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON1) == HIGH) delay(10);   // released before 6 s: really go off
  digitalWrite(PIN_LED, LOW);
  delay(500);

  while (digitalRead(PIN_BUTTON1) != HIGH) delay(50);
  reboot();
}

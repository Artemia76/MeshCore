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
#endif
}

void T1000aBoard::loop() {
#ifdef T1000A_BLE_CONSOLE
  BleConsole.poll();
#endif
  GR551xBoard::loop();
}

// Long press (3 s, UITask) -> "off". Holding on until 6 s restarts into the BLE DFU bootloader instead
// (handled by the platform variant). Soft off for now: GNSS off, LR1110 held in reset, BLE stopped,
// the CPU waits for the next button press and then reboots.
void T1000aBoard::powerOff() {
  digitalWrite(GPS_EN, LOW);
  digitalWrite(P_LORA_RESET, LOW);
  gr551x_ble_stop();

  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON1) == HIGH) delay(10);   // released before 6 s: really go off
  digitalWrite(PIN_LED, LOW);
  delay(500);

  while (digitalRead(PIN_BUTTON1) != HIGH) delay(50);
  reboot();
}

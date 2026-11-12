#include "T1000aBoard.h"

#ifdef T1000A_BLE_CONSOLE
  #include <BleConsole.h>
  #include <helpers/gr551x/SerialBLEInterface.h>

static T1000aBoard* s_board;

static bool consoleCommand(int argc, char** argv) {
  if (!strcmp(argv[0], "bat")) {
    int raw = analogReadMilliVolts(PIN_BATTERY);
    BleConsole.printf("battery: %u mV (MSIO4 %d mV)\r\n", s_board->getBattMilliVolts(), raw);
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

  // LR1110 RF front-end enable, set before the radio is initialised (as in Seeed's firmware)
  pinMode(PIN_LORA_RF_EN, OUTPUT);
  digitalWrite(PIN_LORA_RF_EN, HIGH);

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

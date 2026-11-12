#include "T1000aBoard.h"

#ifdef T1000A_BLE_CONSOLE
  #include <BleConsole.h>
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
#endif
}

void T1000aBoard::loop() {
#ifdef T1000A_BLE_CONSOLE
  BleConsole.poll();
#endif
  GR551xBoard::loop();
}

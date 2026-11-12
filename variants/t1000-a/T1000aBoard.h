#pragma once

#include <helpers/gr551x/GR551xBoard.h>

// Seeed SenseCAP T1000-A: Goodix GR5515 (BLE) + Semtech LR1110 + Airoha AG3335.
// Flashing: over BLE through the stock Seeed bootloader ("start ota" restarts into it).
class T1000aBoard : public GR551xBoard {
public:
  void begin() override;
  void loop() override;
  const char* getManufacturerName() const override { return "Seeed Tracker T1000-A"; }

  // Battery on MSIO4 through a /2 divider (divider enabled by PIN_BATTERY_EN, kept high).
  uint16_t getBattMilliVolts() override {
    int mv = analogReadMilliVolts(PIN_BATTERY);
    return mv > 0 ? (uint16_t)(mv * BATTERY_DIVIDER) : 0;
  }
};

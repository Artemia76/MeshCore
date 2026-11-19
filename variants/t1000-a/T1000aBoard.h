#pragma once

#include <helpers/gr551x/GR551xBoard.h>
#ifdef T1000A_BLE_CONSOLE
  #include "PowerLog.h"
#endif

// Seeed SenseCAP T1000-A: Goodix GR5515 (BLE) + Semtech LR1110 + Airoha AG3335.
// Flashing: over BLE through the stock Seeed bootloader ("start ota" restarts into it).
class T1000aBoard : public GR551xBoard {
  bool _ext_power = false;
  uint32_t _ext_power_at = 0;
  bool _ext_power_valid = false;

  bool holdButton(uint32_t ms);
  bool cablePresent();
  void offBoot(const gr551x_off_info_t& off);
  [[noreturn]] void sleepNow(uint16_t batt_mv);

public:
  void begin() override;
  void loop() override;
  void powerOff() override;
  const char* getManufacturerName() const override { return "Seeed Tracker T1000-A"; }

  // Charging cable on MSIO0 (~2490 mV plugged in, ~35 mV unplugged, measured on the board); read at most once a second
  bool isExternalPowered() override;
  // No light sleep on the charger: keeps the LED animation smooth, and power does not matter there
  void sleep(uint32_t secs) override {
    if (!isExternalPowered()) GR551xBoard::sleep(secs);
  }

#ifdef T1000A_BLE_CONSOLE
  T1000aPowerLog powerLog;   // time spent in each state, read with the "power" console command
  void onBeforeTransmit() override { powerLog.txStart(); }
  void onAfterTransmit() override { powerLog.txEnd(); }
#endif

  // Battery on MSIO4 through a /2 divider (divider enabled by PIN_BATTERY_EN, kept high).
  // GPIO14 enables the battery divider (÷2 on MSIO4). Seeed's firmware drives it high only around each ADC reading:
  // all ten places that set it are followed by a reading, then by a reset to 0, and its LoRa works with it low.
  // Kept low otherwise: the divider would drain the battery continuously.
  static const uint32_t BATT_DIV_SETTLE_MS = 2;
  uint16_t getBattMilliVolts() override {
    digitalWrite(PIN_LORA_RF_EN, HIGH);
    delay(BATT_DIV_SETTLE_MS);
    int mv = analogReadMilliVolts(PIN_BATTERY);
    digitalWrite(PIN_LORA_RF_EN, LOW);
    return mv > 0 ? (uint16_t)(mv * BATTERY_DIVIDER) : 0;
  }
};

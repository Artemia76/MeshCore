#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/KeyValueStore.h>

// Base board for Goodix GR551x (platform-gr551x Arduino core).
// The core provides gr551x_reboot() / gr551x_enter_bootloader(); the board variant of the platform maps them
// to the board's bootloader (e.g. the Seeed BLE DFU bootloader on the T1000-A).
class GR551xBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason = BD_STARTUP_NORMAL;
  unsigned long dfu_at = 0;

public:
  virtual void begin() { startup_reason = BD_STARTUP_NORMAL; }

  void attachDynamicPrefs(KeyValueStore* prefs) { }  // no-op

  uint8_t getStartupReason() const override { return startup_reason; }
  uint16_t getBattMilliVolts() override { return 0; }  // not supported
  float getMCUTemperature() override;                  // die temperature (sense ADC), GR551xBoard.cpp
  const char* getManufacturerName() const override { return "Generic GR551x"; }
  void reboot() override { gr551x_reboot(); }

  // light sleep (WFI) until the next interrupt; the core wakes the loop at least once per second
  void sleep(uint32_t secs) override { gr551x_idle_request(); }

  void loop() override {
    if (dfu_at && millis() >= dfu_at) gr551x_enter_bootloader();
  }

  // "start ota": restart into the bootloader (BLE DFU on the boards supported so far)
  bool startOTAUpdate(const char* id, char reply[]) override {
    dfu_at = millis() + 1000;   // let the reply go out first
    strcpy(reply, "OK - restarting into DFU bootloader");
    return true;
  }
};

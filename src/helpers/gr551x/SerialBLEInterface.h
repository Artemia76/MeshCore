#pragma once

// Companion BLE interface for Goodix GR551x (ROM BLE stack, platform-gr551x Arduino core).
// Exposes the Nordic UART Service (6E400001-...) used by the MeshCore apps: one GATT write = one
// received frame, one notification = one sent frame.

#include "../BaseSerialInterface.h"

#ifndef BLE_TX_POWER
#define BLE_TX_POWER 0
#endif

class SerialBLEInterface : public BaseSerialInterface {
public:
  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  #define FRAME_QUEUE_SIZE  12

  SerialBLEInterface() { }

  /**
   * init the BLE interface.
   * @param prefix   a prefix for the device name
   * @param name  IN/OUT - a name for the device (combined with prefix). If "@@MAC", is modified and returned
   * @param pin_code   the BLE passkey (static: entered on the phone), 0 = no pairing
   */
  void begin(const char* prefix, char* name, uint32_t pin_code);

  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }
  bool isConnected() const override;
  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;

  // called from the BLE stack (interrupt context)
  void onRx(const uint8_t* data, uint16_t len);

  volatile bool _isEnabled = false;
  volatile bool _txBusy = false;

private:
  Frame send_queue[FRAME_QUEUE_SIZE];
  volatile uint8_t send_queue_len = 0;
  Frame recv_queue[FRAME_QUEUE_SIZE];
  volatile uint8_t recv_queue_len = 0;
  unsigned long _last_health_check = 0;

  void clearBuffers();
};

// Prints the recent BLE connection / security events (diagnostics)
void serialBLEDumpLog(Print& out);

#if BLE_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define BLE_DEBUG_PRINT(F, ...) Serial.printf("BLE: " F, ##__VA_ARGS__)
  #define BLE_DEBUG_PRINTLN(F, ...) Serial.printf("BLE: " F "\n", ##__VA_ARGS__)
#else
  #define BLE_DEBUG_PRINT(...) {}
  #define BLE_DEBUG_PRINTLN(...) {}
#endif

#pragma once

// LittleFS on the GR551x internal flash (Adafruit_LittleFS API, as on nRF52 / STM32).
// The flash region is given by the board variant (GR551X_FS_ADDR / GR551X_FS_SIZE in pins_arduino.h).

#include <Arduino.h>
#include "Adafruit_LittleFS.h"

class InternalFileSystem : public Adafruit_LittleFS {
public:
  InternalFileSystem(void);

  // mount, or format then mount if the region does not hold a valid file system
  bool begin(void);
};

extern InternalFileSystem InternalFS;

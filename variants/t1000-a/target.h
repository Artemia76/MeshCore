#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include "T1000aBoard.h"
#include <helpers/radiolib/CustomLR1110Wrapper.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/LocationProvider.h>
#ifdef DISPLAY_CLASS
  #include "NullDisplayDriver.h"
#endif

class T1000aGpsDriver;

class T1000aSensorManager : public SensorManager {
  LocationProvider* _nmea;
  T1000aGpsDriver* _gps_driver;

public:
  T1000aSensorManager(LocationProvider& nmea, T1000aGpsDriver& gps_driver)
    : _nmea(&nmea), _gps_driver(&gps_driver) { }
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  void loop() override;
  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;
  LocationProvider* getLocationProvider() { return _nmea; }
};

#ifdef DISPLAY_CLASS
  extern NullDisplayDriver display;
#endif

extern T1000aBoard board;
extern WRAPPER_CLASS radio_driver;
extern VolatileRTCClock rtc_clock;
extern T1000aSensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();

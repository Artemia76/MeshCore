#pragma once

#include <helpers/sensors/AirohaGpsDriver.h>

// Airoha AG3335 on the T1000-A. Same chip and pin names as the T1000-E, but the GNSS reset line is
// active LOW here (Seeed's firmware pulses it low then leaves it high), and the power-up sequence
// follows Seeed's: EN, 50 ms, VRTC/RESET/SLEEP_INT high, RTC_INT pulse.
class T1000aGpsDriver : public AirohaGpsDriver {
public:
  T1000aGpsDriver(UART_TYPE& serial) : AirohaGpsDriver(serial) { }

  void begin() override {
    NmeaGpsDriver::begin();
    digitalWrite(GPS_EN, HIGH);
    delay(50);
    digitalWrite(GPS_VRTC_EN, HIGH);
    digitalWrite(GPS_RESET, HIGH);
    digitalWrite(GPS_SLEEP_INT, HIGH);
    digitalWrite(GPS_RTC_INT, HIGH);
    delay(3);
    digitalWrite(GPS_RTC_INT, LOW);
  }

  void reset() override {
    digitalWrite(GPS_RESET, LOW);
    delay(10);
    digitalWrite(GPS_RESET, HIGH);
  }
};

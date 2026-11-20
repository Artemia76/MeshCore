#include <Arduino.h>
#include <math.h>
#include "target.h"
#include "T1000aGpsDriver.h"
#include <InternalFileSystem.h>
#include <helpers/sensors/MicroNMEALocationProvider.h>

T1000aBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock rtc_clock;
T1000aGpsDriver gps_driver(Serial1);
MicroNMEALocationProvider nmea(gps_driver, &rtc_clock);
T1000aSensorManager sensors(nmea, gps_driver);

#ifdef DISPLAY_CLASS
  NullDisplayDriver display;
#endif

#ifndef LORA_CR
  #define LORA_CR      5
#endif

// Same RF switch wiring as the T1000-E (LR1110 DIO5..DIO8)
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO7,
  RADIOLIB_LR11X0_DIO8,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6  DIO7  DIO8
  { LR11x0::MODE_STBY,   {LOW,  LOW,  LOW,  LOW  }},
  { LR11x0::MODE_RX,     {HIGH, LOW,  LOW,  HIGH }},
  { LR11x0::MODE_TX,     {HIGH, HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HP,  {LOW,  HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HF,  {LOW,  LOW,  LOW,  LOW  }},
  { LR11x0::MODE_GNSS,   {LOW,  LOW,  HIGH, LOW  }},
  { LR11x0::MODE_WIFI,   {LOW,  LOW,  LOW,  LOW  }},
  END_OF_MODE_TABLE,
};

bool radio_init() {
  SPI.begin();
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER,
                           16, LR11X0_DIO3_TCXO_VOLTAGE);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;
  }

  radio.setCRC(2);
  radio.explicitHeader();
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif
  return true;
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

// ---------------------------------------------------------------- sensors (GPS, battery state of charge, charger)

bool T1000aSensorManager::begin() {
  _gps_driver->initialize();
  return true;
}

// Battery state of charge from its voltage: Li-ion open-circuit curve in 10 % steps, interpolated (reads high
// while charging, the voltage then being above the open-circuit one)
static uint8_t battPercent(uint16_t mv) {
  static const uint16_t ocv[11] = {4190, 4050, 3990, 3890, 3800, 3720, 3670, 3640, 3580, 3460, 3100};  // 100..0 %
  if (mv >= ocv[0]) return 100;
  if (mv <= ocv[10]) return 0;
  int i = 1;
  while (mv < ocv[i]) i++;
  return (uint8_t)(100 - 10 * i + 10 * (mv - ocv[i]) / (ocv[i - 1] - ocv[i]));
}

#define TELEM_CHANNEL_CHARGER  (TELEM_CHANNEL_SELF + 1)
#define TELEM_CHANNEL_ENV      (TELEM_CHANNEL_SELF + 2)   // channel 1 already carries the MCU temperature

bool T1000aSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_BASE) {
    telemetry.addPercentage(TELEM_CHANNEL_SELF, battPercent(board.getBattMilliVolts()));
    // charger: cable plugged in (digital input), charging (switch)
    telemetry.addDigitalInput(TELEM_CHANNEL_CHARGER, board.isExternalPowered() ? 1 : 0);
    telemetry.addSwitch(TELEM_CHANNEL_CHARGER, board.isCharging() ? 1 : 0);
  }
  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    T1000aBoard::Environment e;
    board.readEnvironment(e);
    if (!isnan(e.temp_c)) telemetry.addTemperature(TELEM_CHANNEL_ENV, e.temp_c);
    // 0..100 % like Seeed's firmware, sent as Luminosity so that the app labels it (as on the T1000-E)
    telemetry.addLuminosity(TELEM_CHANNEL_ENV, e.light_pct);
    T1000aBoard::Accel a;
    if (board.readAccel(a)) telemetry.addAccelerometer(TELEM_CHANNEL_ENV, a.g(a.x), a.g(a.y), a.g(a.z));
  }
  if (requester_permissions & TELEM_PERM_LOCATION) {
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  return true;
}

void T1000aSensorManager::loop() {
  static long next_gps_update = 0;
  _nmea->loop();
  if (millis() > next_gps_update) {
    if (_nmea->isGpsOn() && _nmea->hasValidFix()) {
      node_lat = ((double)_nmea->getLatitude()) / 1000000.;
      node_lon = ((double)_nmea->getLongitude()) / 1000000.;
      node_altitude = ((double)_nmea->getAltitude()) / 1000.0;
    }
    next_gps_update = millis() + 1000;
  }
}

int T1000aSensorManager::getNumSettings() const { return 1; }   // "gps" (power switch)

const char* T1000aSensorManager::getSettingName(int i) const {
  return i == 0 ? "gps" : NULL;
}

const char* T1000aSensorManager::getSettingValue(int i) const {
  if (i == 0) return _nmea->isGpsOn() ? "1" : "0";
  return NULL;
}

bool T1000aSensorManager::setSettingValue(const char* name, const char* value) {
  if (strcmp(name, "gps") == 0) {
    _nmea->setGpsOn(strcmp(value, "0") != 0);
    return true;
  }
  return false;
}

// Power off: LR1110 cold sleep (no configuration kept; radio_init() runs again after the reboot)
void t1000aRadioSleep() {
  // RadioLib sends SetSleep without waiting for BUSY low: in the RX duty cycle's sleep phase that NSS edge only
  // wakes the LR1110 and the command is lost. So leave the cycle first.
  radio.leaveDuty();
  radio.sleep(false, 0);
}

// RX duty cycle (console "rxduty", power log). The setting is kept in InternalFS.
#define RXDUTY_FILE "/t1000a_rxduty"

#ifdef T1000A_BLE_CONSOLE   // console "rxduty on|off"
void t1000aRxDutySet(bool on, bool save) {
  radio_driver.setRxDutyCycle(on);
  if (!save) return;
  InternalFS.remove(RXDUTY_FILE);
  Adafruit_LittleFS_Namespace::File f = InternalFS.open(RXDUTY_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (f) { f.write((uint8_t)(on ? '1' : '0')); f.close(); }
}
#endif

// Saved setting, applied once the file system is up (radio_init() runs before InternalFS.begin())
void t1000aRxDutyLoad() {
  Adafruit_LittleFS_Namespace::File f = InternalFS.open(RXDUTY_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ);
  if (!f) return;
  int c = f.read();
  f.close();
  if (c == '0' || c == '1') radio_driver.setRxDutyCycle(c == '1');
}

bool t1000aRxDutyOn() { return radio_driver.isDutyActive(); }
uint16_t t1000aRxDutyShare() { return radio_driver.getRxDutyShare(); }

#ifdef T1000A_BLE_CONSOLE   // debug build: power log and console diagnostics
void t1000aRxDutyInfo(Print& out) {
  out.printf("rxduty: %s, %s", radio_driver.getRxDutyCycle() ? "on" : "off",
             radio_driver.isDutyActive() ? "cycling" : "continuous receive");
  if (radio_driver.isDutyActive()) {
    uint16_t pm = radio_driver.getRxDutyShare();
    out.printf(", listen %lu ms / sleep %lu ms (%u.%u %%)", (unsigned long)(radio_driver.getDutyRxUs() / 1000),
               (unsigned long)(radio_driver.getDutySleepUs() / 1000), pm / 10, pm % 10);
  }
  out.printf(", noise floor %d dBm, rx %lu (errors %lu), radio wake-ups %lu, stuck asleep %lu\r\n",
             radio_driver.getNoiseFloor(), (unsigned long)radio_driver.getPacketsRecv(),
             (unsigned long)radio_driver.getPacketsRecvErrors(), (unsigned long)radio_driver.getRadioWakes(),
             (unsigned long)radio_driver.getStuckWakes());
}

// Packet counters for the power log
void t1000aRadioCounts(uint32_t& rx, uint32_t& tx) {
  rx = radio_driver.getPacketsRecv();
  tx = radio_driver.getPacketsSent();
}

// Console diagnostics ("rxdiag [reset]"): RadioLib calls, main-loop gaps, transmissions
void t1000aRadioDiag(Print& out, bool reset) {
  radio_driver.printDiag(out);
  if (reset) { radio_driver.resetDiag(); out.print("rxdiag: reset\r\n"); }
}

// "sched cpu": representative computation, timed on the board and in the twin
void t1000aSha256(uint8_t* hash, const uint8_t* msg, int len) { mesh::Utils::sha256(hash, 32, msg, len); }

// Console diagnostics ("gps")
void t1000aConsoleGps(Print& out) {
  out.printf("gps: on=%d fix=%d sats=%ld lat=%ld lon=%ld uart_rx=%lu bytes en=%d\r\n", nmea.isGpsOn(), nmea.hasValidFix(),
             (long)nmea.satellitesCount(), (long)nmea.getLatitude(), (long)nmea.getLongitude(), (unsigned long)Serial1.rxBytes(),
             digitalRead(GPS_EN) == HIGH);
  out.printf("gps: sky=%u (max %u in window) gsv=%lu nosky_streak=%u period=%lus %s %lus\r\n", nmea.skyLast(),
             nmea.skyMaxInWindow(), (unsigned long)nmea.gsvCount(), nmea.noskyStreak(), (unsigned long)nmea.getGpsPeriod(),
             nmea.acquiringPeriodically() ? "acquiring for" : "next in",
             (unsigned long)(nmea.acquiringPeriodically() ? nmea.getAcquisitionAge() : nmea.nextAcquisitionIn()));
}
#endif

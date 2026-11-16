#include <Arduino.h>
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

// ---------------------------------------------------------------- sensors (GPS only for now)

bool T1000aSensorManager::begin() {
  _gps_driver->initialize();
  return true;
}

bool T1000aSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
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

void t1000aRxDutySet(bool on, bool save) {
  radio_driver.setRxDutyCycle(on);
  if (!save) return;
  InternalFS.remove(RXDUTY_FILE);
  Adafruit_LittleFS_Namespace::File f = InternalFS.open(RXDUTY_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (f) { f.write((uint8_t)(on ? '1' : '0')); f.close(); }
}

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

void t1000aRxDutyInfo(Print& out) {
  out.printf("rxduty: %s, %s", radio_driver.getRxDutyCycle() ? "on" : "off",
             radio_driver.isDutyActive() ? "cycling" : "continuous receive");
  if (radio_driver.isDutyActive()) {
    uint16_t pm = radio_driver.getRxDutyShare();
    out.printf(", listen %lu ms / sleep %lu ms (%u.%u %%)", (unsigned long)(radio_driver.getDutyRxUs() / 1000),
               (unsigned long)(radio_driver.getDutySleepUs() / 1000), pm / 10, pm % 10);
  }
  out.printf(", noise floor %d dBm, rx %lu (errors %lu), radio wake-ups %lu\r\n", radio_driver.getNoiseFloor(),
             (unsigned long)radio_driver.getPacketsRecv(), (unsigned long)radio_driver.getPacketsRecvErrors(),
             (unsigned long)radio_driver.getRadioWakes());
}

#ifdef T1000A_BLE_CONSOLE   // debug build: power log and console diagnostics
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

// Console diagnostics ("gps")
void t1000aConsoleGps(Print& out) {
  out.printf("gps: on=%d fix=%d sats=%ld lat=%ld lon=%ld uart_rx=%lu octets en=%d\r\n", nmea.isGpsOn(), nmea.hasValidFix(),
             (long)nmea.satellitesCount(), (long)nmea.getLatitude(), (long)nmea.getLongitude(), (unsigned long)Serial1.rxBytes(),
             digitalRead(GPS_EN) == HIGH);
}
#endif

#include "T1000aBoard.h"
#include <math.h>

// Ambient temperature and light, as Seeed's T1000-A firmware reads them (v2.5, 0x0103498C / 0x01034950 and the
// conversions at 0x01033B3C / 0x01033BF0), on the analog circuit that GPIO14 powers:
//   MSIO1 = sensor rail / 2, MSIO2 = thermistor divider (8.25 kOhm to ground, NTC to the rail), MSIO3 = photocell.
// Same circuit, constants and NTC table as the T1000-E (variants/t1000-e/t1000e_sensors.cpp).

static const float NTC_SERIES_OHM = 8250.0f;
static const uint32_t NTC_RES[136] = {   // ohm, -30 .. 105 degC by 1 degC
  113347, 107565, 102116, 96978, 92132, 87559, 83242, 79166, 75316, 71677, 68237, 64991, 61919, 59011,
  56258,  53650,  51178,  48835, 46613, 44506, 42506, 40600, 38791, 37073, 35442, 33892, 32420, 31020,
  29689,  28423,  27219,  26076, 24988, 23951, 22963, 22021, 21123, 20267, 19450, 18670, 17926, 17214,
  16534,  15886,  15266,  14674, 14108, 13566, 13049, 12554, 12081, 11628, 11195, 10780, 10382, 10000,
  9634,   9284,   8947,   8624,  8315,  8018,  7734,  7461,  7199,  6948,  6707,  6475,  6253,  6039,
  5834,   5636,   5445,   5262,  5086,  4917,  4754,  4597,  4446,  4301,  4161,  4026,  3896,  3771,
  3651,   3535,   3423,   3315,  3211,  3111,  3014,  2922,  2834,  2748,  2666,  2586,  2509,  2435,
  2364,   2294,   2228,   2163,  2100,  2040,  1981,  1925,  1870,  1817,  1766,  1716,  1669,  1622,
  1578,   1535,   1493,   1452,  1413,  1375,  1338,  1303,  1268,  1234,  1202,  1170,  1139,  1110,
  1081,   1053,   1026,   999,   974,   949,   925,   902,   880,   858,
};
static const int NTC_T0 = -30;

float T1000aBoard::ntcCelsius(int rail_mv, int ntc_mv) {
  if (ntc_mv <= 0 || rail_mv <= ntc_mv) return NAN;           // open or shorted divider, or rail not powered
  float rt = NTC_SERIES_OHM * rail_mv / ntc_mv - NTC_SERIES_OHM;
  if (rt >= NTC_RES[0] || rt <= NTC_RES[135]) return NAN;     // outside the table (-30 .. 105 degC)
  int i = 1;
  while (rt < NTC_RES[i]) i++;
  return NTC_T0 + (i - 1) + (NTC_RES[i - 1] - rt) / (float)(NTC_RES[i - 1] - NTC_RES[i]);
}

uint8_t T1000aBoard::lightPercent(int light_mv) {   // Seeed: 0..100 % of the photocell range, not lux
  if (light_mv <= 80) return 0;
  if (light_mv >= 2480) return 100;
  return (uint8_t)((light_mv - 80) * 100 / 2400);
}

void T1000aBoard::readEnvironment(Environment& e) {
  // Seeed turns the LED off while reading the photocell; not while charging, where the LED is PWM-driven
  bool led = !isExternalPowered() && digitalRead(PIN_LED) == HIGH;
  if (led) digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LORA_RF_EN, HIGH);
  delay(BATT_DIV_SETTLE_MS);
  int rail = analogReadMilliVolts(PIN_SENSOR_RAIL);
  int ntc = analogReadMilliVolts(PIN_TEMP_SENSOR);
  int light = analogReadMilliVolts(PIN_LIGHT_SENSOR);
  digitalWrite(PIN_LORA_RF_EN, LOW);
  if (led) digitalWrite(PIN_LED, HIGH);
  e.rail_mv = rail > 0 ? rail * 2 : 0;
  e.ntc_mv = ntc;
  e.light_mv = light;
  e.temp_c = ntcCelsius(e.rail_mv, ntc);
  e.light_pct = lightPercent(light);
}

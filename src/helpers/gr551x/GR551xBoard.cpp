#include "GR551xBoard.h"
#include <math.h>
#include <string.h>
#include "gr551x_sdk.h"

extern "C" {
#include "app_adc.h"
#include "gr55xx_sys.h"
}

// Die temperature from the GR551x sense ADC (internal temperature input), as Goodix's SDK example does it
// (projects/peripheral/adc/adc_temp_vbat, gr551x_temp_api.c): single-ended, internal 0.8 V reference, 1.6 MHz,
// 16 samples of which the last 8 are averaged, factory trim (sys_adc_trim_get) with the SDK's fallback values.
float GR551xBoard::getMCUTemperature() {
  static app_adc_params_t p;
  memset(&p, 0, sizeof(p));
  p.init.channel_p  = LL_ADC_INPUT_SRC_TMP;
  p.init.channel_n  = LL_ADC_INPUT_SRC_TMP;
  p.init.input_mode = LL_ADC_INPUT_SINGLE;
  p.init.ref_source = LL_ADC_REF_SRC_BUF_INT;
  p.init.ref_value  = LL_ADC_REF_VALUE_0P8;
  p.init.clock      = LL_ADC_CLK_1P6;
  if (app_adc_init(&p, NULL) != APP_DRV_SUCCESS) return NAN;

  uint16_t raw[16];
  bool ok = app_adc_conversion_sync(raw, 16, 1000) == APP_DRV_SUCCESS;
  app_adc_deinit();
  if (!ok) return NAN;

  uint32_t sum = 0;
  for (int i = 8; i < 16; i++) sum += raw[i];
  double avg = sum / 8.0;

  double t0 = 4979, slope = -4932;
  adc_trim_info_t trim;
  memset(&trim, 0, sizeof(trim));
  if (sys_adc_trim_get(&trim) == SDK_SUCCESS && trim.slope_int_0p8) {
    t0 = trim.adc_temp;
    slope = -(double)trim.slope_int_0p8;
  }
  return (float)(((avg - t0) / slope) / -0.00175 + 25.0);
}

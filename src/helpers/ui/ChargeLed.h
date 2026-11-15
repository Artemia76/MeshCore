#pragma once

#include <math.h>

// Level (0..255, for analogWrite) of a dimmable status LED on external power: sine breathing while charging
// (gamma corrected, never fully dark), steady once charged. Shared by ui-orig and boards that show the charge
// while powered off.
inline int chargeLedLevel(unsigned long now, bool charging, unsigned long period_ms = 4000) {
  if (!charging) return 255;
  const float lo = 0.12f;   // perceived minimum: 1 % duty after gamma
  float ph = (float)(now % period_ms) / (float)period_ms;
  float b = lo + (1.0f - lo) * 0.5f * (1.0f - cosf(2.0f * (float)M_PI * ph));
  int level = (int)(powf(b, 2.2f) * 255.0f + 0.5f);
  return level < 3 ? 3 : level;
}

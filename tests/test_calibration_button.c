#include "calibration_button.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
  calibration_debounce_t s = {0};
  assert(!calibration_debounce_update(&s, true, 1000, 50000));
  assert(!calibration_debounce_update(&s, false, 2000, 50000));
  assert(!calibration_debounce_update(&s, true, 3000, 50000));
  assert(!calibration_debounce_update(&s, true, 52999, 50000));
  assert(calibration_debounce_update(&s, true, 53000, 50000));
  assert(!calibration_debounce_update(&s, true, 1000000, 50000));
  /* A bounced HIGH must not rearm a still-held button. */
  assert(!calibration_debounce_update(&s, false, 1001000, 50000));
  assert(!calibration_debounce_update(&s, true, 1002000, 50000));
  assert(!calibration_debounce_update(&s, true, 1052000, 50000));
  assert(!calibration_debounce_update(&s, false, 1100000, 50000));
  assert(!calibration_debounce_update(&s, false, 1150000, 50000));
  assert(!calibration_debounce_update(&s, true, 1200000, 50000));
  assert(calibration_debounce_update(&s, true, 1250000, 50000));
  calibration_debounce_t boot_low = {0};
  assert(!calibration_debounce_update(&boot_low, true, 0, 50000));
  assert(calibration_debounce_update(&boot_low, true, 50000, 50000));
  puts("Calibration button bounce/hold/rearm/boot-low tests passed");
  return 0;
}

#pragma once
#include <stdbool.h>
#include <stdint.h>

/** Debounce state owned by one input_task() (or one native test).
 * Initialize to zero; timestamps use one monotonic microsecond clock. */
typedef struct {
  bool low;           /**< Last observed logical LOW state. */
  bool delivered;     /**< Request already delivered for this stable LOW. */
  int64_t changed_us; /**< Time of the latest sampled transition, us. */
} calibration_debounce_t;

/** Internal pure helper called by input_task() and native debounce tests.
 * s must be non-NULL, now monotonic us and debounce_us positive. Accept one
 * request per stable LOW; stable HIGH rearms without exiting maintenance.
 * Return true only on acceptance. No allocation, hardware or synchronization;
 * callers must serialize updates to the same state. */
static inline bool calibration_debounce_update(calibration_debounce_t *s,
                                               bool low, int64_t now,
                                               int64_t debounce_us) {
  if (low != s->low) {
    s->low = low;
    s->changed_us = now;
  }
  if (now - s->changed_us < debounce_us)
    return false;
  if (!low) {
    s->delivered = false;
    return false;
  }
  if (s->delivered)
    return false;
  s->delivered = true;
  return true;
}

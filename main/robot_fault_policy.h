#pragma once
#include "esp_drv8833.h"

/* Application policy requested for this hardware. Context is temporal
 * association supplied by the driver, not a diagnosis of the electrical cause.
 * All ULT events are informational, including unclassified/persistent LOW.
 */
/** Internal classification used by fault() and native policy tests.
 * Return true for recognized driver transition associations. Does not diagnose
 * electrical cause or alter the informational-only continuation policy. */
static inline bool
robot_ult_context_known(esp_drv8833_fault_context_t context) {
  switch (context) {
  case ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION:
  case ESP_DRV8833_FAULT_CONTEXT_WAKE:
  case ESP_DRV8833_FAULT_CONTEXT_MOTOR_START:
  case ESP_DRV8833_FAULT_CONTEXT_COAST:
  case ESP_DRV8833_FAULT_CONTEXT_REVERSAL:
  case ESP_DRV8833_FAULT_CONTEXT_BRAKE:
    return true;
  case ESP_DRV8833_FAULT_CONTEXT_NONE:
  default:
    return false;
  }
}

/** Internal policy used by fault() and tests; always false, even for unknown
 * or continuously LOW ULT. state is not dereferenced and may be NULL. Sensor,
 * output and timing errors retain their separate component policies. */
static inline bool robot_ult_requires_stop(const esp_drv8833_state_t *state) {
  (void)state;
  return false;
}

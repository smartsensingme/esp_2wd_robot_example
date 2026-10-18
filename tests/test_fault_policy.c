#include "robot_fault_policy.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
  esp_drv8833_state_t state = {0};
  assert(!robot_ult_requires_stop(&state));
  state.fault_latched = true;
  assert(!robot_ult_requires_stop(&state));
  const esp_drv8833_fault_context_t known[] = {
      ESP_DRV8833_FAULT_CONTEXT_INITIALIZATION,
      ESP_DRV8833_FAULT_CONTEXT_WAKE,
      ESP_DRV8833_FAULT_CONTEXT_MOTOR_START,
      ESP_DRV8833_FAULT_CONTEXT_COAST,
      ESP_DRV8833_FAULT_CONTEXT_REVERSAL,
      ESP_DRV8833_FAULT_CONTEXT_BRAKE};
  for (unsigned i = 0; i < sizeof(known) / sizeof(known[0]); ++i) {
    assert(robot_ult_context_known(known[i]));
    state.last_fault_event.context = known[i];
    state.fault_latched = true;
    state.fault_event_pending = true;
    assert(!robot_ult_requires_stop(&state));
    state.fault_latched = false;
    state.fault_event_pending = false;
    state.fault_pin_low =
        true; /* Same assertion remains classified after ACK. */
    assert(!robot_ult_requires_stop(&state));
  }
  assert(!robot_ult_context_known(ESP_DRV8833_FAULT_CONTEXT_NONE));
  state.last_fault_event.context = ESP_DRV8833_FAULT_CONTEXT_NONE;
  assert(!robot_ult_requires_stop(&state));
  state.fault_pin_low = false;
  assert(!robot_ult_requires_stop(&state));
  state.fault_latched = true; /* Unknown transient remains informational. */
  assert(!robot_ult_requires_stop(&state));
  state.last_fault_event.context = (esp_drv8833_fault_context_t)999;
  assert(!robot_ult_requires_stop(&state));
  for (unsigned flags = 0; flags < 8; ++flags) {
    state.fault_pin_low = (flags & 1) != 0;
    state.fault_latched = (flags & 2) != 0;
    state.fault_event_pending = (flags & 4) != 0;
    assert(!robot_ult_requires_stop(&state));
  }
  puts("Application ULT informational-only policy tests passed");
}

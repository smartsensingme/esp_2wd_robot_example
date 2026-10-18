#pragma once
/** @file calibration_input.h
 * Application-only active-LOW maintenance input and optional LED.
 * One worker owns debounce/GPIO; console and PID supervisor exchange atomic
 * flags. There is no shutdown API: initialize once and keep alive until reset.
 */
#include "esp_err.h"
#include <stdbool.h>

/** Called internally by app_main() once, after hardware initialization.
 * Configure allowed free GPIOs, input pull-up and optional inactive LED; create
 * a priority-1/2048-byte worker polling every 10 ms. Return ESP_ERR_INVALID_ARG
 * for unavailable/colliding pins, GPIO driver errors or ESP_ERR_NO_MEM on task
 * creation. Partial GPIO configuration is not rolled back. Task context only.
 */
esp_err_t calibration_input_start(void);

/** Internal: run_pid_test() and service_calibration_request() observe the
 * latched atomic request. Safe across tasks; reading does not acknowledge it.
 */
bool calibration_input_requested(void);

/** Internal: service_calibration_request() calls after successful service
 * entry. Clear the request and latch active LED state. Does not change robot
 * mode; releasing input/rearming debounce never exits maintenance. Task context
 * only.
 */
void calibration_input_acknowledge(void);

#include "calibration_input.h"
#include "board_config.h"
#include "calibration_button.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdatomic.h>

static atomic_bool requested;
static atomic_bool active;

/* Board's unused exposed GPIOs; exclude I2C, bridge, flash and native USB. */
/** Internal: calibration_input_start() checks the board free-pin whitelist.
 * This intentionally excludes occupied I2C/bridge/native USB/flash connections.
 */
static bool available_pin(int pin) {
  return pin == 4 || pin == 5 || pin == 6 || pin == 20 || pin == 21;
}

/** Internal lifetime worker created by calibration_input_start().
 * Sample monotonic us, debounce LOW/HIGH, latch a request and optionally toggle
 * the LED every 250 ms while acknowledged active. Never touches motor/I2C/NVS.
 * Priority 1 polling latency is scheduling-dependent; no hard entry deadline.
 */
static void input_task(void *unused) {
  (void)unused;
  calibration_debounce_t debounce = {0};
  for (;;) {
    int64_t now = esp_timer_get_time();
    if (calibration_debounce_update(
            &debounce, gpio_get_level(CONFIG_ROBOT_CALIBRATION_GPIO) == 0, now,
            (int64_t)CONFIG_ROBOT_CALIBRATION_DEBOUNCE_MS * 1000))
      atomic_store(&requested, true);
#if CONFIG_ROBOT_CALIBRATION_LED_GPIO >= 0
    bool on = atomic_load(&active) && ((now / 250000) & 1);
    gpio_set_level(CONFIG_ROBOT_CALIBRATION_LED_GPIO,
                   on ? CONFIG_ROBOT_CALIBRATION_LED_ACTIVE_LEVEL
                      : !CONFIG_ROBOT_CALIBRATION_LED_ACTIVE_LEVEL);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/** Internal board setup called once by app_main(); see calibration_input.h.
 * Validate pins before any GPIO mutation; configure input/optional LED before
 * launching input_task(). There is no resource teardown or retry lifecycle.
 */
esp_err_t calibration_input_start(void) {
  int pin = CONFIG_ROBOT_CALIBRATION_GPIO;
  int led = CONFIG_ROBOT_CALIBRATION_LED_GPIO;
  if (!available_pin(pin) || (led >= 0 && (!available_pin(led) || led == pin)))
    return ESP_ERR_INVALID_ARG;
  gpio_config_t input = {.pin_bit_mask = 1ULL << pin,
                         .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE,
                         .pull_down_en = GPIO_PULLDOWN_DISABLE,
                         .intr_type = GPIO_INTR_DISABLE};
  esp_err_t error = gpio_config(&input);
  if (error != ESP_OK)
    return error;
#if CONFIG_ROBOT_CALIBRATION_LED_GPIO >= 0
  gpio_config_t output = {.pin_bit_mask = 1ULL << led,
                          .mode = GPIO_MODE_OUTPUT,
                          .intr_type = GPIO_INTR_DISABLE};
  error = gpio_set_level(led, !CONFIG_ROBOT_CALIBRATION_LED_ACTIVE_LEVEL);
  if (error == ESP_OK)
    error = gpio_config(&output);
  if (error != ESP_OK)
    return error;
#endif
  return xTaskCreate(input_task, "cal_input", 2048, NULL, 1, NULL) == pdPASS
             ? ESP_OK
             : ESP_ERR_NO_MEM;
}

/** Internal atomic observation used by PID/console; see public board header.
 */
bool calibration_input_requested(void) {
  return atomic_load(&requested);
}

/** Internal console acknowledgement after confirmed entry; see board header.
 */
void calibration_input_acknowledge(void) {
  atomic_store(&requested, false);
  atomic_store(&active, true);
}

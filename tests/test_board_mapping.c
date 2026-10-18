#include "esp_drv8833.h"
#include "board_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  /* A forward raw increment from each physical sensor must normalize positive.
   * Historical calibration storage must stay attached to that same sensor. */
  assert(ROBOT_LEFT_BRIDGE == ESP_DRV8833_BRIDGE_B);
  assert(ROBOT_LEFT_I2C_ADDRESS == 0x06);
  assert(ROBOT_RIGHT_I2C_ADDRESS == 0x46);
  assert(12 * ROBOT_LEFT_ENCODER_SIGN > 0);
  assert(-12 * ROBOT_RIGHT_ENCODER_SIGN > 0);
  assert(strcmp(ROBOT_LEFT_LUT_NAMESPACE, "robot_right") == 0);
  assert(strcmp(ROBOT_RIGHT_LUT_NAMESPACE, "robot_left") == 0);
  puts("Physical wheel/encoder/LUT mapping tests passed");
  return 0;
}

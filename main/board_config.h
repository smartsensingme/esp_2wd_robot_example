#pragma once

/* docs/pins.txt. User-verified side mapping: motor 1/A=right, motor 2/B=left.
 * Positive bridge commands already move both wheels forward. */
/** Shared encoder I2C data GPIO; also connected to the onboard LED. */
#define ROBOT_SDA_GPIO 8
/** Shared encoder I2C clock GPIO. */
#define ROBOT_SCL_GPIO 9
/** DRV8833 bridge A input 1 GPIO (physical RIGHT). */
#define ROBOT_IN1_GPIO 0
/** DRV8833 bridge A input 2 GPIO (physical RIGHT). */
#define ROBOT_IN2_GPIO 1
/** DRV8833 bridge B input 1 GPIO (physical LEFT). */
#define ROBOT_IN3_GPIO 2
/** DRV8833 bridge B input 2 GPIO (physical LEFT). */
#define ROBOT_IN4_GPIO 3
/** EEP/nSLEEP output GPIO; verify module jumper before relying on sleep. */
#define ROBOT_SLEEP_GPIO 10
/** Shared ULT/nFAULT input GPIO; informational policy only. */
#define ROBOT_FAULT_GPIO 7
/** Physical LEFT 7-bit encoder address. */
#define ROBOT_LEFT_I2C_ADDRESS 0x06
/** Physical RIGHT 7-bit encoder address. */
#define ROBOT_RIGHT_I2C_ADDRESS 0x46
/* Historical NVS names follow sensor identity, not their old side labels.
 * Preserve installed tables: robot_right was captured from 0x06, robot_left
 * from 0x46 before the physical-side correction on 2026-10-17. */
/** Historical NVS namespace belonging to physical LEFT sensor 0x06. */
#define ROBOT_LEFT_LUT_NAMESPACE "robot_right"
/** Historical NVS namespace belonging to physical RIGHT sensor 0x46. */
#define ROBOT_RIGHT_LUT_NAMESPACE "robot_left"
/** Physical LEFT bridge selection; opposite channel is RIGHT. */
#define ROBOT_LEFT_BRIDGE ESP_DRV8833_BRIDGE_B
/** Bridge PWM carrier frequency, Hz. */
#define ROBOT_PWM_HZ 20000
/** Both encoder device clock requests, Hz. */
#define ROBOT_I2C_HZ 400000
/** Normalize increasing LEFT counts to forward wheel motion. */
#define ROBOT_LEFT_ENCODER_SIGN 1
/** Normalize decreasing RIGHT counts to forward wheel motion. */
#define ROBOT_RIGHT_ENCODER_SIGN (-1)

/* Geometry measured by user on 2026-10-17; limits below are test choices,
 * not measured motor ratings. Sensor mounting reverses the right raw direction;
 * normalize feedback once in the robot, without changing bridge polarity. */
/** Internal C initializer used by ROBOT_CONFIG_INITIALIZER. sensor_sign must be
 * +1 or -1. Radius is meters, ratio is sensor turns/wheel turn, speed is rad/s,
 * ramp rad/s² and duty percent. Limits are bench choices, not motor ratings. */
#define ROBOT_WHEEL_CONFIG_INITIALIZER(sensor_sign)                            \
  {.radius_m = 0.03305f,                                                       \
   .encoder_revolutions_per_wheel_revolution = 1.0f,                           \
   .max_speed_rad_s = 6.0f,                                                    \
   .max_acceleration_rad_s2 = 4.0f,                                            \
   .max_duty_percent = 75.0f,                                                  \
   .encoder_sign = (sensor_sign),                                              \
   .motor_sign = 1}
/** Internal geometry initializer used by initialize_hardware() and
 * run_pid_test(). Track is meters; period and brake duration are microseconds.
 * This creates value configuration only, without initializing hardware or
 * commanding motion. */
#define ROBOT_CONFIG_INITIALIZER                                               \
  {.left = ROBOT_WHEEL_CONFIG_INITIALIZER(ROBOT_LEFT_ENCODER_SIGN),            \
   .right = ROBOT_WHEEL_CONFIG_INITIALIZER(ROBOT_RIGHT_ENCODER_SIGN),          \
   .wheel_separation_m = 0.1186f,                                              \
   .control_period_us = 5000,                                                  \
   .brake_max_duration_us = 100000}

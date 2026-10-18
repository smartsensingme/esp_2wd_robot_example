/** @file main.c
 * Board integration and synchronous USB console for the C3 bench application.
 * Hardware adapters are borrowed by one controller or maintenance owner at a
 * time. See README.md and docs/control-timing.md for current commands/limits.
 */
#include "board_config.h"
#include "calibration_input.h"
#include "driver/i2c_master.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_2wd_robot_calibration.h"
#include "esp_2wd_robot_calibration_protocol.h"
#include "esp_2wd_robot_control.h"
#include "esp_angle_lut.h"
#include "esp_drv8833.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mt6701.h"
#include "nvs_flash.h"
#include "robot_fault_policy.h"
#include "sdkconfig.h"
#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

/* Application owns all driver/bus/LUT handles. Robot borrows this static
 * context and serializes all callbacks from its one owner task. No automatic
 * drive. */
static struct {
  i2c_master_bus_handle_t bus;
  i2c_master_dev_handle_t devices[2];
  mt6701_dev_t sensors[2];
  esp_angle_lut_handle_t lut[2];
  esp_drv8833_handle_t bridge;
  esp_2wd_robot_handle_t robot;
  esp_2wd_robot_calibration_handle_t calibration;
  esp_2wd_robot_calibration_protocol_handle_t protocol;
  atomic_uint ult_known, ult_unknown, ult_last_context, ult_telemetry_errors;
  bool have_ult_capture;
  uint32_t ult_last_edges;
  int64_t ult_last_timestamp;
} hardware;

/* Keep startup failures queryable even when no controller can be created. */
static esp_err_t startup_error = ESP_OK;
static const char *startup_resource = "nvs";

/**
 * Read the selected native 14-bit encoder with the driver's 1-ms timeout.
 * Internal I/O callback: called by the robot owner, maintenance acquisition and
 * run_status(). On success, write the transaction midpoint in monotonic us;
 * on error the timestamp is not updated. Borrowed hardware stays alive. No log,
 * allocation or retry is added; exclusive ownership must be established first.
 */
static esp_err_t read_encoder(void *context, bool left, uint16_t *raw,
                              int64_t *timestamp) {
  (void)context;
  unsigned i = left ? 0 : 1;
  int64_t before = esp_timer_get_time();
  esp_err_t error = mt6701_read_raw_angle(&hardware.sensors[i], raw);
  int64_t after = esp_timer_get_time();
  if (error == ESP_OK)
    *timestamp = before + (after - before) / 2;
  return error;
}

/**
 * Apply the selected native-coordinate LUT before unwrap/sign conversion.
 * Internal callback used by the controller created in run_pid_test(); raw and
 * result are counts modulo 16384. Disabled/unloaded behavior belongs to the LUT
 * driver. Do not mutate that LUT while the controller is alive.
 */
static uint16_t correct_angle(void *context, bool left, uint16_t raw) {
  (void)context;
  return esp_angle_lut_instance_apply(hardware.lut[left ? 0 : 1], raw);
}

/**
 * Write signed electrical duty percentages through the shared bridge.
 * Internal callback used by controller and maintenance acquisition. The robot
 * supplies LEFT,RIGHT; the driver expects RIGHT,LEFT. Preserve signs here;
 * return the driver's output/wake error for the caller's stop policy.
 */
static esp_err_t drive(void *context, float left, float right) {
  (void)context;
  /* Driver is RIGHT,LEFT; robot callback is LEFT,RIGHT. Signs are applied
   * once in the robot, so driver polarity inversion remains false. */
  return esp_drv8833_set_speed(hardware.bridge, right, left);
}

/**
 * Request both-bridge electrical braking, returning the driver error.
 * Internal callback used by controller and calibration stop sequences. It does
 * not wait for wheel rest; the component bounds braking before coast.
 */
static esp_err_t brake(void *context) {
  (void)context;
  return esp_drv8833_brake(hardware.bridge);
}

/**
 * Request both outputs high impedance; optionally assert hardware sleep.
 * Internal callback used by controller/maintenance lifecycle and stop handling.
 * sleep=false keeps the bridge awake; success is an API result, not a measured
 * mechanical stop. Caller retains resources on teardown errors.
 */
static esp_err_t coast(void *context, bool sleep) {
  (void)context;
  return sleep ? esp_drv8833_coast(hardware.bridge)
               : esp_drv8833_coast_awake(hardware.bridge);
}

/**
 * Collect shared ULT telemetry without requesting a robot fault.
 * Internal callback used by control and maintenance acquisition. The valid
 * active output is always false under this application's informational policy;
 * monitoring/acknowledgement errors increment telemetry counters and return OK.
 * One exclusive hardware owner updates event identity; atomic counters may be
 * observed by report(). Driver context is association, not electrical
 * diagnosis.
 */
static esp_err_t fault(void *context, bool *active) {
  (void)context;
  *active = false;
  esp_drv8833_state_t state;
  esp_err_t error = esp_drv8833_get_state(hardware.bridge, &state);
  if (error != ESP_OK) {
    atomic_fetch_add(&hardware.ult_telemetry_errors, 1);
    return ESP_OK; /* Informational monitoring cannot stop the owner. */
  }
  *active = robot_ult_requires_stop(&state);
  /* Count accepted captures once, not repeated polls of the same latch.
   * Formatting stays in the reporter. These are lifetime application counters,
   * sampled asynchronously there, not counts aligned to its diagnostic window.
   */
  if (state.fault_latched &&
      (!hardware.have_ult_capture ||
       hardware.ult_last_edges != state.last_fault_event.falling_edges ||
       hardware.ult_last_timestamp != state.last_fault_event.timestamp_us)) {
    hardware.have_ult_capture = true;
    hardware.ult_last_edges = state.last_fault_event.falling_edges;
    hardware.ult_last_timestamp = state.last_fault_event.timestamp_us;
    atomic_uint *counter =
        robot_ult_context_known(state.last_fault_event.context)
            ? &hardware.ult_known
            : &hardware.ult_unknown;
    unsigned count = atomic_load(counter);
    if (count != UINT32_MAX)
      atomic_store(counter, count + 1);
    atomic_store(&hardware.ult_last_context,
                 (unsigned)state.last_fault_event.context);
  }
  /* Rearm telemetry only after the worker has copied the event. No ULT level,
   * context or acknowledgement failure is transferred to the robot fault latch.
   */
  if (state.fault_latched && !state.fault_event_pending &&
      esp_drv8833_clear_fault(hardware.bridge) != ESP_OK)
    atomic_fetch_add(&hardware.ult_telemetry_errors, 1);
  return ESP_OK;
}

/* This callback runs only on the lower-priority reporter, never the owner. */
/**
 * Format a copied state and asynchronous lifetime ULT counters.
 * Internal callback configured by run_pid_test() (periodic reports disabled)
 * and called directly there after its stop/join attempt. payload must point to
 * a live esp_2wd_robot_state_t for this call; other callback arguments are
 * unused. Formatting is outside the periodic owner. This is not a per-cycle
 * recorder.
 */
static void report(const esp_rt_diag_snapshot_t *diagnostics,
                   const void *payload, esp_rt_diag_window_class_t window,
                   void *context) {
  (void)diagnostics;
  (void)window;
  (void)context;
  const esp_2wd_robot_state_t *state = payload;
  ESP_LOGI("robot_state",
           "mode=%u valid=%d left_raw=%u right_raw=%u "
           "left_speed=%.3f right_speed=%.3f rad/s "
           "left_duty=%.1f right_duty=%.1f%% "
           "error=%s",
           (unsigned)state->mode, state->sample_valid, state->raw[0],
           state->raw[1], (double)state->measured.left_rad_s,
           (double)state->measured.right_rad_s,
           (double)state->left_duty_percent, (double)state->right_duty_percent,
           esp_err_to_name(state->last_error));
  ESP_LOGI("robot_ult",
           "bridge=shared informational=1 accepted known=%u unknown=%u "
           "last_context=%u telemetry_errors=%u",
           atomic_load(&hardware.ult_known), atomic_load(&hardware.ult_unknown),
           atomic_load(&hardware.ult_last_context),
           atomic_load(&hardware.ult_telemetry_errors));
  ESP_LOGI("robot_encoders", "left=0x%02x:%s right=0x%02x:%s",
           ROBOT_LEFT_I2C_ADDRESS, esp_err_to_name(state->encoder_error[0]),
           ROBOT_RIGHT_I2C_ADDRESS, esp_err_to_name(state->encoder_error[1]));
}

static bool calibration_usb_read(void *, uint8_t *, size_t, uint32_t);
static bool calibration_usb_write(void *, const uint8_t *, size_t, uint32_t);
static bool calibration_usb_drain(void *, uint32_t);

/**
 * Build application-owned resources before any movement can be requested.
 * Internal: app_main() calls once. Initialize NVS without erasing on failure,
 * probe both sensors, load per-sensor LUTs, create coasting bridge and
 * maintenance service/protocol. startup_resource identifies the last setup
 * phase/failed side. Return the first relevant ESP error; partially created
 * handles remain in hardware for cleanup(). No controller or motion reference
 * is created here.
 */
static esp_err_t initialize_hardware(void) {
  esp_err_t error = nvs_flash_init();
  if (error != ESP_OK)
    return error; /* Never erase calibration automatically. */
  startup_resource = "i2c_bus";
  i2c_master_bus_config_t bus = {.i2c_port = I2C_NUM_0,
                                 .sda_io_num = ROBOT_SDA_GPIO,
                                 .scl_io_num = ROBOT_SCL_GPIO,
                                 .clk_source = I2C_CLK_SRC_DEFAULT,
                                 .glitch_ignore_cnt = 7,
                                 .flags.enable_internal_pullup = true};
  if ((error = i2c_new_master_bus(&bus, &hardware.bus)) != ESP_OK)
    return error;
  const unsigned addresses[2] = {ROBOT_LEFT_I2C_ADDRESS,
                                 ROBOT_RIGHT_I2C_ADDRESS};
  const char *names[2] = {ROBOT_LEFT_LUT_NAMESPACE, ROBOT_RIGHT_LUT_NAMESPACE};
  const char *sides[2] = {"left", "right"};
  esp_err_t first_sensor_error = ESP_OK;
  const char *first_sensor_side = NULL;
  for (unsigned i = 0; i < 2; i++) {
    ESP_LOGI("robot_hw", "MT6701 %s addr=0x%02x SDA=%d SCL=%d bus=%u Hz",
             sides[i], addresses[i], ROBOT_SDA_GPIO, ROBOT_SCL_GPIO,
             ROBOT_I2C_HZ);
    error = i2c_master_probe(hardware.bus, addresses[i], 20);
    if (error != ESP_OK) {
      ESP_LOGE("robot_hw",
               "MT6701 %s addr=0x%02x address probe failed: %s (0x%x)",
               sides[i], addresses[i], esp_err_to_name(error), (unsigned)error);
      if (first_sensor_error == ESP_OK) {
        first_sensor_error = error;
        first_sensor_side = sides[i];
      }
      continue;
    }
    i2c_device_config_t device = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                  .device_address = addresses[i],
                                  .scl_speed_hz = ROBOT_I2C_HZ};
    error =
        i2c_master_bus_add_device(hardware.bus, &device, &hardware.devices[i]);
    if (error == ESP_OK)
      error = mt6701_init_with_timeout(&hardware.sensors[i],
                                       hardware.devices[i], 1);
    if (error != ESP_OK) {
      ESP_LOGE("robot_hw",
               "MT6701 %s addr=0x%02x initialization/read failed: %s (0x%x)",
               sides[i], addresses[i], esp_err_to_name(error), (unsigned)error);
      if (first_sensor_error == ESP_OK) {
        first_sensor_error = error;
        first_sensor_side = sides[i];
      }
      continue;
    }
    ESP_LOGI("robot_hw", "MT6701 %s addr=0x%02x communication OK", sides[i],
             addresses[i]);
  }
  /* Test both configured addresses before returning failure. Never create the
   * bridge/control owner when either sensor failed initialization. */
  if (first_sensor_error != ESP_OK) {
    startup_resource = first_sensor_side;
    return first_sensor_error;
  }
  startup_resource = "lut";
  for (unsigned i = 0; i < 2; i++) {
    if ((error = esp_angle_lut_create(names[i], &hardware.lut[i])) != ESP_OK)
      return error;
    esp_angle_lut_status_t status;
    esp_angle_lut_instance_get_status(hardware.lut[i], &status);
    if (status.full_scale_counts != MT6701_COUNTS_PER_REVOLUTION)
      return ESP_ERR_INVALID_STATE;
  }
  esp_drv8833_config_t bridge = ESP_DRV8833_CONFIG_DEFAULT();
  startup_resource = "bridge";
  bridge.in1_gpio = ROBOT_IN1_GPIO;
  bridge.in2_gpio = ROBOT_IN2_GPIO;
  bridge.in3_gpio = ROBOT_IN3_GPIO;
  bridge.in4_gpio = ROBOT_IN4_GPIO;
  bridge.sleep_gpio = ROBOT_SLEEP_GPIO;
  bridge.fault_gpio = ROBOT_FAULT_GPIO;
  bridge.left_bridge = ROBOT_LEFT_BRIDGE;
  bridge.pwm_freq_hz = ROBOT_PWM_HZ;
  error = esp_drv8833_create(&bridge, &hardware.bridge);
  if (error != ESP_OK)
    return error;
  esp_2wd_robot_calibration_config_t calibration;
  startup_resource = "calibration_service";
  esp_2wd_robot_calibration_defaults(&calibration);
  calibration.io = (esp_2wd_robot_io_t){.read_encoder = read_encoder,
                                        .drive = drive,
                                        .brake = brake,
                                        .coast = coast,
                                        .fault = fault};
  calibration.lut[0] = hardware.lut[0];
  calibration.lut[1] = hardware.lut[1];
  error = esp_2wd_robot_calibration_create(&calibration, &hardware.calibration);
  if (error != ESP_OK)
    return error;
  esp_2wd_robot_calibration_protocol_config_t protocol = {
      .calibration = hardware.calibration,
      .geometry = ROBOT_CONFIG_INITIALIZER,
      .read_exact = calibration_usb_read,
      .write_exact = calibration_usb_write,
      .drain = calibration_usb_drain};
  return esp_2wd_robot_calibration_protocol_create(&protocol,
                                                   &hardware.protocol);
}

/**
 * Release partially initialized hardware in reverse borrowing order.
 * Internal: called by app_main() after initialize_hardware() fails. Stop/join
 * controller and calibration before releasing borrowed bridge/LUT/I2C
 * resources. A failed owner teardown retains resources; driver-delete errors
 * are logged without blindly retrying a potentially consumed handle. Startup
 * error stays queryable through the USB console. Not a general concurrent
 * shutdown API.
 */
static void cleanup(void) {
  if (hardware.robot) {
    if (esp_2wd_robot_destroy(hardware.robot) != ESP_OK) {
      ESP_LOGE("robot",
               "Owner still closing; retaining borrowed hardware resources");
      return;
    }
    hardware.robot = NULL;
  }
  if (hardware.protocol) {
    esp_2wd_robot_calibration_protocol_destroy(hardware.protocol);
    hardware.protocol = NULL;
  }
  if (hardware.calibration) {
    if (esp_2wd_robot_calibration_destroy(hardware.calibration) != ESP_OK) {
      ESP_LOGE("robot",
               "Calibration still closing; retaining borrowed hardware");
      return;
    }
    hardware.calibration = NULL;
  }
  if (hardware.bridge) {
    /* Driver delete may consume its handle on cleanup errors; no blind retry.
     */
    esp_err_t error = esp_drv8833_delete(hardware.bridge);
    if (error != ESP_OK)
      ESP_LOGE("robot", "Bridge cleanup: %s", esp_err_to_name(error));
    hardware.bridge = NULL;
  }
  for (unsigned i = 0; i < 2; i++) {
    if (hardware.lut[i]) {
      esp_angle_lut_destroy(hardware.lut[i]);
      hardware.lut[i] = NULL;
    }
    if (hardware.devices[i]) {
      i2c_master_bus_rm_device(hardware.devices[i]);
      hardware.devices[i] = NULL;
    }
  }
  if (hardware.bus) {
    i2c_del_master_bus(hardware.bus);
    hardware.bus = NULL;
  }
}

/* Transport adapter only: all acquisition/state/safety belongs to the
 * component. */
static esp_2wd_robot_state_t pid_samples[50];

/**
 * Run the explicitly requested bounded two-wheel PID bench experiment.
 * Internal: dispatch_command() calls in normal mode only. Create a 200-Hz
 * owner, wait up to one second for valid position/velocity, arm asynchronously
 * and submit +3 rad/s per wheel. Observe for at most five seconds after
 * submission, recording at most 50 value snapshots; a GPIO request or error
 * ends observation early. Request stop, wait 150 ms, attempt destroy, then
 * format stored data. A failed destroy retains hardware.robot; logging then is
 * not proof the owner has joined. Caller serializes console/maintenance access.
 * LUT/NVS data is not modified.
 */
static void run_pid_test(void) {
  ESP_LOGI("pid_test",
           "Starting PID trial (+3 rad/s both wheels, 5 s, max duty=75%%, "
           "left/right Kalman sigma=0.05 deg accel_std=5 rad/s^2)...");
  const esp_2wd_robot_config_t geometry = ROBOT_CONFIG_INITIALIZER;
  esp_2wd_robot_control_config_t config;
  esp_err_t error = esp_2wd_robot_control_defaults(&geometry, &config);
  if (error == ESP_OK) {
    config.report_callback = report;
    config.enable_reports = false;
    config.use_kalman_velocity = true;
    /* Board trial selected from both recorded traces and synthetic lag tests.
     * Keep reusable defaults and raw LUT calibration unchanged. */
    config.kalman_position_std_rad[0] = config.kalman_position_std_rad[1] =
        0.05f * 0.0174532925199f;
    config.kalman_acceleration_std_rad_s2[0] =
        config.kalman_acceleration_std_rad_s2[1] = 5.0f;
    config.left_pid.ki = config.right_pid.ki = 20.0f;
    config.sensor_error_fault_count = 3;
    config.max_sample_gap_us = 15000;
    esp_2wd_robot_io_t io = {.read_encoder = read_encoder,
                             .correct_angle = correct_angle,
                             .drive = drive,
                             .brake = brake,
                             .coast = coast,
                             .fault = fault};
    error = esp_2wd_robot_create(&config, &io, &hardware.robot);
  }
  if (error != ESP_OK) {
    ESP_LOGE("pid_test", "Control initialization: %s", esp_err_to_name(error));
    return;
  }

  esp_2wd_robot_state_t state = {0};
  const char *phase = "wait_feedback";
  /* create() confirms task/timer startup, not acquisition of the angle pair
   * and derivative baseline. Keep coasting while the owner establishes them.
   * Its sensor/fault policies still run; never clear a fault here. */
  int64_t feedback_deadline = esp_timer_get_time() + 1000000;
  while (error == ESP_OK) {
    error = esp_2wd_robot_get_state(hardware.robot, &state);
    if (error != ESP_OK)
      break;
    if (state.mode != ESP_2WD_ROBOT_STOPPED ||
        state.last_stop_error != ESP_OK) {
      error = state.last_error != ESP_OK        ? state.last_error
              : state.last_stop_error != ESP_OK ? state.last_stop_error
                                                : ESP_ERR_INVALID_STATE;
      break;
    }
    if (calibration_input_requested()) {
      error = ESP_ERR_INVALID_STATE;
      break;
    }
    if (state.sample_valid && state.velocity_valid)
      break;
    if (esp_timer_get_time() >= feedback_deadline) {
      error = ESP_ERR_TIMEOUT;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (error == ESP_OK)
    phase = "start";
  if (error == ESP_OK)
    error = esp_2wd_robot_start(hardware.robot);
  if (error == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(20));
    error = esp_2wd_robot_get_state(hardware.robot, &state);
    if (error == ESP_OK && (state.mode != ESP_2WD_ROBOT_RUNNING ||
                            state.last_command_error != ESP_OK))
      error = ESP_ERR_INVALID_STATE;
  }
  if (error == ESP_OK) {
    phase = "reference";
    error = esp_2wd_robot_set_wheel_speeds(hardware.robot, 3.0f, 3.0f);
  }
  if (error == ESP_OK)
    phase = "run";

  unsigned count = 0;
  int64_t start = esp_timer_get_time();
  int64_t next_sample = start;
  while (error == ESP_OK && !calibration_input_requested() &&
         esp_timer_get_time() - start < 5000000) {
    error = esp_2wd_robot_get_state(hardware.robot, &state);
    if (error != ESP_OK)
      break;
    if (state.mode != ESP_2WD_ROBOT_RUNNING ||
        state.last_command_error != ESP_OK) {
      error =
          state.last_error != ESP_OK ? state.last_error : ESP_ERR_INVALID_STATE;
      break;
    }
    if (esp_timer_get_time() >= next_sample && count < 50) {
      pid_samples[count++] = state;
      next_sample += 100000;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  esp_err_t stop_error = esp_2wd_robot_stop(hardware.robot);
  vTaskDelay(pdMS_TO_TICKS(150));
  esp_2wd_robot_state_t final = state;
  esp_err_t final_error = esp_2wd_robot_get_state(hardware.robot, &final);

  /* Destroy robot controller instance only; shared hardware handles remain
   * active! */
  esp_err_t destroy_error = esp_2wd_robot_destroy(hardware.robot);
  if (destroy_error == ESP_OK)
    hardware.robot = NULL; /* Retain ownership on a failed stop/join. */

  ESP_LOGI("pid_test",
           "finished result=%s phase=%s stop_request=%s final_snapshot=%s "
           "samples=%u sensor_errors=%lu missed=%lu deadlines=%lu "
           "max_processing_us=%lu max_lateness_us=%lu",
           esp_err_to_name(error), phase, esp_err_to_name(stop_error),
           esp_err_to_name(final_error), count,
           (unsigned long) final.sensor_errors,
           (unsigned long) final.missed_releases,
           (unsigned long) final.deadline_misses,
           (unsigned long) final.max_processing_us,
           (unsigned long) final.max_release_lateness_us);

  for (unsigned i = 0; i < count; i++) {
    const esp_2wd_robot_state_t *s = &pid_samples[i];
    ESP_LOGI(
        "pid_test",
        "sample=%u t_us=%lld "
        "left_target=%.3f left_ramped=%.3f left_speed=%.3f left_duty=%.2f "
        "right_target=%.3f right_ramped=%.3f right_speed=%.3f right_duty=%.2f "
        "left_diff_speed=%.3f right_diff_speed=%.3f "
        "estimator=kalman_cv speed_unit=rad/s duty_unit=percent "
        "velocity_valid=%d",
        i, (long long)s->timestamp_us, (double)s->target.left_rad_s,
        (double)s->ramped.left_rad_s, (double)s->measured.left_rad_s,
        (double)s->left_duty_percent, (double)s->target.right_rad_s,
        (double)s->ramped.right_rad_s, (double)s->measured.right_rad_s,
        (double)s->right_duty_percent, (double)s->differentiated.left_rad_s,
        (double)s->differentiated.right_rad_s, s->velocity_valid);
  }
  report(NULL, &final, ESP_RT_DIAG_WINDOW_QUIET, NULL);
}

/**
 * Read both raw sensors and report per-sensor LUT state without driving.
 * Internal: dispatch_command() calls synchronously from the console, with no
 * active acquisition in the normal completed flow. Errors accompany raw values;
 * raw output on a failed read is not valid feedback. LUTs remain unchanged.
 */
static void run_status(void) {
  uint16_t raw[2] = {0};
  int64_t t[2] = {0};
  esp_err_t e0 = read_encoder(NULL, true, &raw[0], &t[0]);
  esp_err_t e1 = read_encoder(NULL, false, &raw[1], &t[1]);
  ESP_LOGI("status", "MT6701 left  (0x%02x): %s, raw=%u (%.2f deg)",
           ROBOT_LEFT_I2C_ADDRESS, esp_err_to_name(e0), raw[0],
           (double)raw[0] * 360.0 / MT6701_COUNTS_PER_REVOLUTION);
  ESP_LOGI("status", "MT6701 right (0x%02x): %s, raw=%u (%.2f deg)",
           ROBOT_RIGHT_I2C_ADDRESS, esp_err_to_name(e1), raw[1],
           (double)raw[1] * 360.0 / MT6701_COUNTS_PER_REVOLUTION);
  for (unsigned i = 0; i < 2; i++) {
    esp_angle_lut_status_t lut_stat = {0};
    esp_angle_lut_instance_get_status(hardware.lut[i], &lut_stat);
    ESP_LOGI("status",
             "LUT %s: loaded=%d enabled=%d generation=%lu crc32=0x%08lx",
             i == 0 ? "left " : "right", lut_stat.loaded, lut_stat.enabled,
             (unsigned long)lut_stat.generation,
             (unsigned long)lut_stat.payload_crc32);
  }
}

/**
 * Accumulate exactly count bytes from nonblocking native-driver stdin.
 * Internal protocol adapter: registered by initialize_hardware(). dest is a
 * caller-owned writable buffer of count bytes; context is unused. On EOF, clear
 * stream status, wait 2 ms and fail when elapsed time exceeds timeout_ms. The
 * timeout is checked on empty reads; continuous input is bounded by count.
 * Return false on incomplete input; consumed bytes are not rolled back.
 */
static bool calibration_usb_read(void *context, uint8_t *dest, size_t count,
                                 uint32_t timeout_ms) {
  (void)context;
  size_t received = 0;
  int64_t start_time = esp_timer_get_time();
  int64_t timeout_us = (int64_t)timeout_ms * 1000;
  while (received < count) {
    int ch = getchar();
    if (ch == EOF) {
      clearerr(stdin);
      if (esp_timer_get_time() - start_time > timeout_us) {
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }
    dest[received++] = (uint8_t)ch;
  }
  return true;
}

/* Same native-driver path as the reference transport's usb_send_all().
 * Never send LUT bytes through the VFS character writer: it may silently drop
 * bytes after its short console timeout, even when fwrite reports success.
 * Maintenance only, with the control owner stopped and no USB producer active.
 */
/**
 * Send count binary bytes using checked native USB driver writes.
 * Internal protocol adapter: registered by initialize_hardware(). src remains
 * valid for the call; context is unused. Flush console text first, accumulate
 * partial writes in chunks <=4096 and enforce a monotonic timeout_ms budget for
 * driver writes. Flush time is outside that explicit budget. Return false on
 * flush/write failure or expiry; a partial frame may already have reached host.
 * Maintenance only: no competing USB producer is permitted.
 */
static bool calibration_usb_write(void *context, const uint8_t *src,
                                  size_t count, uint32_t timeout_ms) {
  (void)context;
  if (fflush(stdout) != 0)
    return false;
  int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
  while (count > 0) {
    size_t chunk = count > 4096 ? 4096 : count;
    int64_t remaining = deadline - esp_timer_get_time();
    if (remaining <= 0)
      return false;
    int written = usb_serial_jtag_write_bytes(
        src, chunk, pdMS_TO_TICKS((uint32_t)((remaining + 999) / 1000)));
    if (written <= 0)
      return false;
    src += written;
    count -= (size_t)written;
  }
  return true;
}

/**
 * Flush console text and wait for queued native USB TX completion.
 * Internal protocol adapter registered by initialize_hardware(); context
 * unused. Return true only when fflush succeeds and the driver's timeout_ms
 * wait does. This does not prove host consumption and fflush is outside the
 * driver timeout.
 */
static bool calibration_usb_drain(void *context, uint32_t timeout_ms) {
  (void)context;
  return fflush(stdout) == 0 &&
         usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

/**
 * Print current human/host commands and flush the prompt.
 * Internal: run_console() and dispatch_command(help) call in console context.
 * This describes operations; it does not select maintenance or start a motor.
 */
static void print_menu(void) {
  printf("\n=========================================================\n"
         "ESP 2WD Robot - Interactive & Host Protocol Console\n"
         "Commands:\n"
         "  cal left / cal right   : Multi-plateau calibration "
         "(70%%->90%%->70%%)\n"
         "  pid                    : Run 5 s closed-loop PID trial (+3 rad/s)\n"
         "  status                 : Show MT6701 readings & LUT status\n"
         "  help                   : Display this menu\n"
         "Host protocol commands:\n"
         "  CAL STATUS [left|right]\n"
         "  CAL START <left|right>\n"
         "  CAL WRITE <left|right> <bins> <full_scale> <crc32_hex>\n"
         "  CAL READ <left|right>\n"
         "  CAL ENABLE <left|right>\n"
         "  CAL DISABLE <left|right>\n"
         "  CAL CLEAR <left|right>\n"
         "  ESTIMATE START <left|right> [sigma_deg accel_std]\n"
         "=========================================================\n"
         "> ");
  fflush(stdout);
}

/**
 * Query whether the borrowed maintenance service has latched entry.
 * Internal: service_calibration_request() and dispatch_command() use this after
 * successful startup. This boolean does not itself authorize concurrent I/O.
 */
static bool calibration_mode(void) {
  return esp_2wd_robot_calibration_is_active(hardware.calibration);
}

/* Only the command task changes mode, after the controller has stopped/joined.
 */
/**
 * Accept an atomic GPIO request only after exclusive ownership is available.
 * Internal: run_console() and dispatch_command() call; run_pid_test() merely
 * observes the request and stops. On startup failure do nothing. Entry
 * delegates controller stop/join/coast to the service; an error leaves the
 * request pending for a later console iteration. Acknowledge/enable the LED
 * only after success.
 */
static void service_calibration_request(void) {
  if (startup_error != ESP_OK)
    return;
  if (!calibration_input_requested())
    return;
  if (!calibration_mode()) {
    esp_err_t error =
        esp_2wd_robot_calibration_enter(hardware.calibration, &hardware.robot);
    if (error != ESP_OK)
      return;
    printf("\nMODE calibration\n");
    fflush(stdout);
  }
  calibration_input_acknowledge();
}

/**
 * Route one mutable NUL-terminated console line synchronously.
 * Internal: run_console() calls after a line terminator. Trim
 * spaces/terminators, report retained startup errors first, service GPIO entry,
 * then delegate robot protocol commands before application
 * ping/pid/status/help. Maintenance rejects PID; command text alone cannot
 * select maintenance. Return false on protocol transport failure so no prompt
 * contaminates an incomplete binary frame.
 */
static bool dispatch_command(char *line) {
  while (*line == ' ')
    line++;
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r' ||
                     line[len - 1] == '\n')) {
    line[--len] = '\0';
  }
  if (len == 0)
    return true;

  if (startup_error != ESP_OK) {
    printf("ERR command=STARTUP code=%s resource=%s "
           "message=hardware_initialization_failed\n",
           esp_err_to_name(startup_error), startup_resource);
    fflush(stdout);
    return true;
  }

  service_calibration_request();
  bool handled = false;
  esp_err_t protocol_error = esp_2wd_robot_calibration_protocol_dispatch(
      hardware.protocol, line, &handled);
  /* Keep truncated frames free of prompts. */
  if (protocol_error != ESP_OK || handled)
    return protocol_error == ESP_OK;
  if ((calibration_mode() || calibration_input_requested()) &&
      strcasecmp(line, "pid") == 0) {
    printf("ERR command=PID message=reset_to_exit_calibration_mode\n");
    fflush(stdout);
    return true;
  }

  if (strcasecmp(line, "ping") == 0) {
    printf("OK command=PING\n");
    fflush(stdout);
  } else if (strcasecmp(line, "pid") == 0) {
    run_pid_test();
  } else if (strcasecmp(line, "status") == 0) {
    run_status();
  } else if (strcasecmp(line, "help") == 0) {
    print_menu();
  } else {
    printf("ERR command=UNKNOWN line='%s' message=type_help_for_options\n",
           line);
    fflush(stdout);
  }
  return true;
}

/**
 * Own command input forever, including after retained startup failures.
 * Internal: app_main() calls once. Buffer at most 255 characters, accept CR/LF
 * or one CRLF terminator, silently discard excess line characters, and poll
 * GPIO requests between reads. Empty reads delay 10 ms. Commands execute
 * synchronously; PID/capture therefore do not concurrently process console
 * input. A suppressed prompt on transport failure does not provide binary
 * stream resynchronization.
 */
static void run_console(bool print_menu_now) {
  char line[256];
  unsigned len = 0;
  bool previous_cr = false;
  if (print_menu_now) {
    print_menu();
  } else {
    printf("> ");
    fflush(stdout);
  }

  for (;;) {
    service_calibration_request();
    int ch = getchar();
    if (ch == EOF) {
      clearerr(stdin);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    /* IDF Monitor can send CR for Enter; Octave sends LF. Accept both and
     * consume CRLF as one terminator without dispatching a command twice. */
    if (ch == '\n' && previous_cr) {
      previous_cr = false;
      continue;
    }
    previous_cr = ch == '\r';
    if (previous_cr)
      ch = '\n';
    if (ch == '\n') {
      line[len] = '\0';
      bool show_prompt = len == 0 || dispatch_command(line);
      if (show_prompt) {
        printf("> ");
        fflush(stdout);
      }
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = (char)ch;
    }
  }
}

/**
 * ESP-IDF entry point; called externally by the framework once at startup.
 * Install native USB TX4096/RX1024 before hardware setup; configure nonblocking
 * stdin and LF endings so the same transport carries console and binary
 * payloads. On setup failure cleanup partial handles and retain a queryable
 * startup error. On success start the input worker and enter run_console().
 * USB/GPIO worker setup uses ESP_ERROR_CHECK; those failures abort rather than
 * using STARTUP. No boot-time reference/motor trial is submitted. Resources
 * live until reset.
 */
void app_main(void) {
  /* The reference uses the interrupt-driven USB driver with software rings.
   * Console RX also uses this driver so it does not compete for the HW FIFO.
   * Keep getchar nonblocking for the finite console/payload waits.
   */
  usb_serial_jtag_driver_config_t usb_config = {.tx_buffer_size = 4096,
                                                .rx_buffer_size = 1024};
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
  usb_serial_jtag_vfs_use_driver();
  int stdin_flags = fcntl(fileno(stdin), F_GETFL);
  ESP_ERROR_CHECK(stdin_flags < 0 || fcntl(fileno(stdin), F_SETFL,
                                           stdin_flags | O_NONBLOCK) < 0
                      ? ESP_FAIL
                      : ESP_OK);

  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);

  startup_error = initialize_hardware();
  if (startup_error != ESP_OK) {
    ESP_LOGE("robot", "Hardware initialization resource=%s: %s",
             startup_resource, esp_err_to_name(startup_error));
    cleanup();
    /* No GPIO mode selection or hardware commands after partial startup.
     * The console reports the retained error instead of silently disappearing.
     */
    run_console(false);
    return;
  }

  ESP_ERROR_CHECK(calibration_input_start());
  printf("MODE normal calibration_gpio=%d active_level=0 debounce_ms=%d\n",
         CONFIG_ROBOT_CALIBRATION_GPIO, CONFIG_ROBOT_CALIBRATION_DEBOUNCE_MS);
  fflush(stdout);
  run_console(true);
}

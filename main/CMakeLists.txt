idf_component_register(SRCS "main.c" "board/calibration_input.c"
    INCLUDE_DIRS "." "board"
    REQUIRES esp_driver_gpio esp_2wd_robot esp_drv8833 esp-mt6701 esp_angle_lut nvs_flash esp_timer esp_rt_diagnostics esp_rt_diagnostics_reporter esp_driver_usb_serial_jtag vfs)

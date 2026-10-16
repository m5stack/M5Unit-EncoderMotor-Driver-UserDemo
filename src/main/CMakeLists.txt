idf_component_register(
    SRCS
        "main.cpp"
        "app/app_encoder_motor.cpp"
        "app/app_motor_ui.c"
        "app/motor_assets/motor_digits_96.c"
    INCLUDE_DIRS "." "app"
    REQUIRES
        m5stack_tab5
        m5_unit_motor_driver
        esp_driver_i2c
        esp_driver_usb_serial_jtag
)

# Generated LVGL fonts use the component include directory directly.
target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_LVGL_H_INCLUDE_SIMPLE)

idf_component_register(SRCS "m5_unit_motor_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES freertos)

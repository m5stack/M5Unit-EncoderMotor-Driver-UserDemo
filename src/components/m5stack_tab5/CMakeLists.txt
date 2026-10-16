
idf_component_register(
    SRCS
        "m5stack_tab5.c"
        "esp_lcd_st7123.c"
    INCLUDE_DIRS "include"
    INCLUDE_DIRS "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES
        driver
        esp_lcd
        esp_lcd_st7121
        esp_codec_dev
        esp_lvgl_port
        esp_driver_ledc
        esp_driver_i2s
        esp_driver_i2c
        spiffs
        fatfs
	    esp_lcd_ili9881c
    PRIV_REQUIRES
)

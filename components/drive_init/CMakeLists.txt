idf_component_register(
    SRCS "driver_init.c"
    INCLUDE_DIRS "."
    REQUIRES    lvgl 
                driver 
                esp_lcd espressif__esp_lcd_touch 
                atanisoft__esp_lcd_touch_xpt2046
)


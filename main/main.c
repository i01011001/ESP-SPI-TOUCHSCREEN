#include "freertos/projdefs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <string.h>

#include "lvgl_demo_ui.h"
#include "driver_init.h"

#define LCD_H_RES 320
#define LCD_V_RES 240
#define LCD_HOST SPI2_HOST
#define TAG "esp_lcd"

void app_main(void)
{
    lcd_driver_init();
    lvgl_init();
    touch_driver_init();
    touch_init();


    xTaskCreate(lvgl_task, "lvgl_task", 4096, NULL, 5, NULL);
    lv_button_test();
}


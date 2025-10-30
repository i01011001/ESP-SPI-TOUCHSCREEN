#include "driver_init.h"
#include "core/lv_obj_pos.h"
#include "display/lv_display.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>

#include "esp_lcd_touch_xpt2046.h"
#include "indev/lv_indev.h"


#define LCD_H_RES 320
#define LCD_V_RES 240
#define LCD_HOST SPI2_HOST
#define TOUCH_HOST SPI3_HOST

#define TAG "esp_lcd"

static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
static esp_lcd_panel_io_handle_t touch_io_handle = NULL;
static esp_lcd_panel_handle_t lcd_panel_handle = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

static void flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
static uint32_t get_time(void);
static void touch_task(void* arg);
static void touch_interrupt_cb(esp_lcd_touch_t* touch_panel);
static void touch_input_init();

static QueueHandle_t queueHandle;
lv_display_t* display = NULL;

void lvgl_init(void)
{
    // Initialize LVGL
    lv_init();
    lv_tick_set_cb(get_time);

    // Create display
    display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // Allocate draw buffer - make it static to ensure it persists
    static lv_color_t buf[LCD_H_RES * LCD_V_RES / 10 * 2];

    lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // // // Connect ESP panel handle to LVGL display
    lv_display_set_user_data(display, lcd_panel_handle);
    lv_display_set_flush_cb(display, flush_cb);

    // Clear the screen first
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), 0);
    //
    // // Force initial render
    lv_refr_now(display);
}

void lvgl_task(void* pvParameter)
{
    ESP_LOGI(TAG, "LVGL task started");

    while (1)
    {
        uint32_t time_till_next = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(time_till_next > 0 ? time_till_next : 20));
    }
}

void lcd_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD");

    // Configure backlight GPIO
    gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << 21};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(21, 0); // Turn off initially

    // Configure SPI bus
    spi_bus_config_t lcd_spi_config = {
        .sclk_io_num = 14,
        .mosi_io_num = 13,
        .miso_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &lcd_spi_config, SPI_DMA_CH_AUTO));

    // Configure LCD IO
    esp_lcd_panel_io_spi_config_t io_config = {.dc_gpio_num = 2,
                                               .cs_gpio_num = 15,
                                               .pclk_hz = 40 * 1000 * 1000,
                                               .lcd_cmd_bits = 8,
                                               .lcd_param_bits = 8,
                                               .spi_mode = 0,
                                               .trans_queue_depth = 10,
                                               .flags = {
                                                   .lsb_first = 0,
                                               }};
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &lcd_io_handle));

    // Configure LCD panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &lcd_panel_handle));

    // Initialize panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));

    // Configure display orientation (adjust these if display is rotated/mirrored wrong)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel_handle, false));
    //
    // Gap settings for ST7789 (may need adjustment based on your specific display)
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_panel_handle, 0, 0));

    // Turn on display
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_handle, true));

    // Turn on backlight
    gpio_set_level(21, 1);

    ESP_LOGI(TAG, "LCD initialization complete");
}

void touch_driver_init(void)
{
    static spi_bus_config_t touch_xpt2056_buscfg = {
        .sclk_io_num = 25,
        .mosi_io_num = 32,
        .miso_io_num = 39,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_HOST, &touch_xpt2056_buscfg, SPI_DMA_CH_AUTO));
}

void touch_init()
{

    void IRAM_ATTR touch_interrupt_cb(esp_lcd_touch_t * touch_panel);
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(33);
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) TOUCH_HOST, &tp_io_config, &touch_io_handle);

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = 36,
        // .interrupt_callback = touch_interrupt_cb,
        .flags =
            {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io_handle, &tp_cfg, &touch_handle));

    touch_input_init();


    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
    esp_lcd_touch_read_data(touch_pad);
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void touch_input_init()
{
    static lv_indev_t *indev;
    indev = lv_indev_create(); // Input device driver (Touch)
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, touch_handle);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);
}

static void flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    lcd_panel_handle = lv_display_get_user_data(disp);

    int w = area->x2 - area->x1 + 1;
    int h = area->y2 - area->y1 + 1;

    // Swap RGB565 byte order for ST7789
    lv_draw_sw_rgb565_swap(px_map, w * h);

    // Draw to LCD
    esp_lcd_panel_draw_bitmap(lcd_panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);

    lv_display_flush_ready(disp);
}

static uint32_t get_time(void)
{
    return esp_timer_get_time() / 1000;
}

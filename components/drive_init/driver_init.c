#include "driver_init.h"
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

#define TAG "esp_lcd"

#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_HOST SPI2_HOST
#define TOUCH_HOST SPI3_HOST

#define LCD_SCLK_PIN 14
#define LCD_MOSI_PIN 13
#define LCD_MISO_PIN 12
#define LCD_DC_PIN 2
#define LCD_CS_PIN 15
#define LCD_BACKLIGHT_PIN 21

#define TOUCH_SCLK_PIN 25
#define TOUCH_MOSI_PIN 32
#define TOUCH_MISO_PIN 39
#define TOUCH_INT_PIN 36
#define TOUCH_CS_PIN 33

static lv_display_t* display;
static esp_lcd_panel_io_handle_t lcd_io_handle;
static esp_lcd_panel_io_handle_t touch_io_handle;
static esp_lcd_panel_handle_t lcd_panel_handle;
static esp_lcd_touch_handle_t touch_handle;
static esp_lcd_touch_handle_t touch_pad;

static void setup_timer();
static void lvgl_touch_cb(lv_indev_t* indev, lv_indev_data_t* data);
static void touch_input_init();
static void lv_tick_task(void* arg);
static void flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);

void lcd_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD");

    spi_bus_config_t lcd_spi_config = {
        .sclk_io_num = LCD_SCLK_PIN,
        .mosi_io_num = LCD_MOSI_PIN,
        .miso_io_num = LCD_MISO_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC_PIN,
        .cs_gpio_num = LCD_CS_PIN,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    // Configure backlight GPIO
    gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << 21};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(21, 0); // Turn off initially

    // Configure SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &lcd_spi_config, SPI_DMA_CH_AUTO));

    // Configure LCD IO
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &lcd_io_handle));

    // Configure LCD panel
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &lcd_panel_handle));

    // Initialize panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));

    // Configure display orientation (adjust these if display is rotated/mirrored wrong)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel_handle, false));
    //
    // Gap settings for ST7789 (may need adjustment based on your specific display)
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_panel_handle, 0, 0));

    // Turn on display
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_handle, true));

    // Turn on backlight
    gpio_set_level(LCD_BACKLIGHT_PIN, 1);

    ESP_LOGI(TAG, "LCD initialization complete");
}

void lvgl_init(void)
{
    lv_init();
    setup_timer();

    display = lv_display_create(LCD_H_RES, LCD_V_RES);

    size_t draw_buffer_sz = LCD_V_RES * 20 * sizeof(lv_color16_t);
    void* buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void* buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);

    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Connect ESP panel handle to LVGL display
    lv_display_set_user_data(display, lcd_panel_handle);
    lv_display_set_flush_cb(display, flush_cb);

    // Clear the screen first
    lv_obj_clean(lv_screen_active());
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), 0);

    // // Force initial render
    lv_refr_now(display);
}

void lvgl_task(void* pvParameter)
{
    ESP_LOGI(TAG, "LVGL task started");

    while (1)
    {
        uint32_t time_till_next = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(time_till_next > 0 ? time_till_next : 5));
    }
}

void touch_driver_init(void)
{
    spi_bus_config_t touch_xpt2056_buscfg = {
        .sclk_io_num = TOUCH_SCLK_PIN,
        .mosi_io_num = TOUCH_MOSI_PIN,
        .miso_io_num = TOUCH_MISO_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_HOST, &touch_xpt2056_buscfg, SPI_DMA_CH_AUTO));
}

void touch_init()
{
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = TOUCH_INT_PIN,
        // .interrupt_callback = touch_interrupt_cb,
        .flags =
            {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
    };
    void IRAM_ATTR touch_interrupt_cb(esp_lcd_touch_t * touch_panel);
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(TOUCH_CS_PIN);
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) TOUCH_HOST, &tp_io_config, &touch_io_handle);

    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io_handle, &tp_cfg, &touch_handle));

    touch_input_init();

    ESP_LOGI(TAG, "Initialize touch controller XPT2046");
}

static void setup_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {.callback = &lv_tick_task, .name = "lv_tick"};
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, 1000); // 1000 us = 1 ms
}

static void lvgl_touch_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_read_data(touch_pad);
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = LCD_H_RES - 1 - touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        esp_rom_printf("%d, %d\n", data->point.x, data->point.y);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void touch_input_init()
{
    static lv_indev_t* indev;
    indev = lv_indev_create(); // Input device driver (Touch)
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, touch_handle);
    touch_pad = lv_indev_get_user_data(indev);
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

static void lv_tick_task(void* arg)
{
    lv_tick_inc(1); // increment LVGL tick by 1 ms
}

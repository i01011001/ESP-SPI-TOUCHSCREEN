#ifndef __LVGL_EXAMPLE_H__
#define __LVGL_EXAMPLE_H__

#include "core/lv_obj.h"
#include "core/lv_obj_pos.h"
#include "core/lv_obj_style_gen.h"
#include "display/lv_display.h"
#include "esp_rom_sys.h"
#include "lv_api_map_v8.h"
#include "lvgl.h"
#include "misc/lv_area.h"
#include "misc/lv_color.h"
#include "widgets/label/lv_label.h"
#include "widgets/slider/lv_slider.h"

static lv_obj_t* btn;
static lv_display_rotation_t rotation = LV_DISP_ROTATION_0;
void create_button(lv_align_t align, lv_event_cb_t cb);
static void create_button_xy(int32_t pos_x, int32_t pos_y, int32_t size_x, int32_t size_y, lv_event_cb_t cb);

static void btn_cb(lv_event_t* e)
{
    lv_display_t* disp = lv_event_get_user_data(e);
    rotation++;
    if (rotation > LV_DISP_ROTATION_270)
    {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);
}
static void set_angle(void* obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

void lvgl_demo_ui(lv_display_t* disp)
{
    lv_obj_t* scr = lv_display_get_screen_active(disp);

    btn = lv_button_create(scr);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text_static(lbl, LV_SYMBOL_REFRESH " ROTATE");
    lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    /*Button event*/
    lv_obj_add_event_cb(btn, btn_cb, LV_EVENT_CLICKED, disp);

    /*Create an Arc*/
    lv_obj_t* arc = lv_arc_create(scr);
    lv_arc_set_rotation(arc, 270);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE); /*To not allow adjusting by click*/
    lv_obj_center(arc);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, arc);
    lv_anim_set_exec_cb(&a, set_angle);
    lv_anim_set_duration(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); /*Just for the demo*/
    lv_anim_set_repeat_delay(&a, 500);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_start(&a);
}

int cnt = 0;
static void btn_event_cb(lv_event_t* e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* btn = lv_event_get_target_obj(e);

    if (code == LV_EVENT_PRESSED)
    {
        // Change to grey when pressed
        lv_obj_set_style_bg_color(btn, lv_color_make(150, 150, 150), LV_PART_MAIN);
    }
    else if (code == LV_EVENT_RELEASED || code == LV_EVENT_CLICKED)
    {
        // Revert when released
        lv_obj_set_style_bg_color(btn, lv_color_make(255, 255, 255), LV_PART_MAIN);
    }
}

/**
 * Create a button with a label and react on click event.
 */
void lv_button_test(void)
{
    const int cols = 4;
    const int rows = 6;
    const int gap_x = 5;
    const int gap_y = 5;

    const int btn_w = (240 - (gap_x * (cols - 1))) / cols; // 56
    const int btn_h = (360 - (gap_y * (rows - 1))) / rows; // 55

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            int x = col * (btn_w + gap_x);
            int y = row * (btn_h + gap_y);
            create_button_xy(x, y, btn_w, btn_h, btn_event_cb);
        }
    }
}

void lv_test_slider()
{
    lv_obj_t* slider = lv_slider_create(lv_screen_active());
    lv_obj_set_size(slider, 100, 8);
    lv_obj_set_align(slider, LV_ALIGN_TOP_MID);

    lv_obj_t* label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "hello");
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_obj_set_style_text_color(label, (lv_color_t){255,255,255}, LV_STATE_DEFAULT);

}

void create_button(lv_align_t align, lv_event_cb_t cb)
{

    lv_obj_t* btn = lv_button_create(lv_screen_active()); /*Add a button the current screen*/
    lv_obj_set_size(btn, 60, 60);                         /*Set its size*/
    lv_obj_add_event_cb(btn, cb, LV_EVENT_ALL, NULL);     /*Assign a callback to the button*/
    lv_obj_set_align(btn, align);
    lv_obj_set_style_bg_color(btn, lv_color_make(255, 255, 255), LV_PART_MAIN);

    // lv_obj_t* label = lv_label_create(btn); /*Add a label to the button*/
    // lv_label_set_text(label, "Button 000"); /*Set the labels text*/
    // lv_obj_center(label);

    lv_area_t coord;
    lv_obj_get_coords(btn, &coord);
    esp_rom_printf("%d,%d\n", coord.x1, coord.y1);
}

static void create_button_xy(int32_t pos_x, int32_t pos_y, int32_t size_x, int32_t size_y, lv_event_cb_t cb)
{

    lv_obj_t* btn = lv_button_create(lv_screen_active()); /*Add a button the current screen*/
    lv_obj_set_size(btn, size_x, size_y);                 /*Set its size*/
    lv_obj_add_event_cb(btn, cb, LV_EVENT_ALL, NULL);     /*Assign a callback to the button*/
    lv_obj_set_pos(btn, pos_x, pos_y);
    lv_obj_set_style_bg_color(btn, lv_color_make(255, 255, 255), LV_PART_MAIN);

    // lv_obj_t* label = lv_label_create(btn); /*Add a label to the button*/
    // lv_label_set_text(label, "Button 000"); /*Set the labels text*/
    // lv_obj_center(label);

    lv_area_t coord;
    lv_obj_get_coords(btn, &coord);
    esp_rom_printf("%d,%d\n", coord.x1, coord.y1);
}

void lvgl_hello()
{
    // Create UI widgets
    lv_obj_t* label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "el phy congro");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_center(label);
    lv_obj_set_ext_click_area(label, 200);
}

#endif

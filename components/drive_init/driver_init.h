#ifndef __DRIVER_INIT_H__
#define __DRIVER_INIT_H__

void lvgl_init(void);
void lvgl_task(void* pvParameter);
void lcd_driver_init(void);
void touch_driver_init(void);
void touch_init();

#endif

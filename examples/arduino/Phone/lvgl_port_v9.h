/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <Arduino.h>
#include <ESP_Panel_Library.h>
#include <lvgl.h>

#ifndef LVGL_PORT_DISP_WIDTH
#   if defined(ESP_PANEL_LCD_WIDTH)
#       define LVGL_PORT_DISP_WIDTH             (ESP_PANEL_LCD_WIDTH)
#   elif defined(ESP_PANEL_BOARD_WIDTH)
#       define LVGL_PORT_DISP_WIDTH             (ESP_PANEL_BOARD_WIDTH)
#   else
#       define LVGL_PORT_DISP_WIDTH             (320)
#   endif
#endif
#ifndef LVGL_PORT_DISP_HEIGHT
#   if defined(ESP_PANEL_LCD_HEIGHT)
#       define LVGL_PORT_DISP_HEIGHT            (ESP_PANEL_LCD_HEIGHT)
#   elif defined(ESP_PANEL_BOARD_HEIGHT)
#       define LVGL_PORT_DISP_HEIGHT            (ESP_PANEL_BOARD_HEIGHT)
#   else
#       define LVGL_PORT_DISP_HEIGHT            (240)
#   endif
#endif
#define LVGL_PORT_TICK_PERIOD_MS                (2)
#define LVGL_PORT_BUFFER_MALLOC_CAPS            (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define LVGL_PORT_BUFFER_SIZE                   (LVGL_PORT_DISP_WIDTH * 20)
#define LVGL_PORT_BUFFER_NUM                    (2)
#define LVGL_PORT_TASK_MAX_DELAY_MS             (500)
#define LVGL_PORT_TASK_MIN_DELAY_MS             (2)
#define LVGL_PORT_TASK_STACK_SIZE               (6 * 1024)
#define LVGL_PORT_TASK_PRIORITY                 (2)
#define LVGL_PORT_TASK_CORE                     (ARDUINO_RUNNING_CORE)

#ifdef __cplusplus
extern "C" {
#endif

bool lvgl_port_init(ESP_PanelLcd *lcd, ESP_PanelTouch *tp);
bool lvgl_port_lock(int timeout_ms);
bool lvgl_port_unlock(void);
lv_display_t *lvgl_port_get_display(void);
lv_indev_t *lvgl_port_get_touch(void);

#ifdef __cplusplus
}
#endif

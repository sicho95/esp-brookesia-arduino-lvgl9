/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <Arduino.h>
#include <lvgl.h>

#define WAVESHARE_I2C_SDA              (15)
#define WAVESHARE_I2C_SCL              (14)

#define WAVESHARE_LCD_CS               (12)
#define WAVESHARE_LCD_SCLK             (38)
#define WAVESHARE_LCD_SIO0             (4)
#define WAVESHARE_LCD_SIO1             (5)
#define WAVESHARE_LCD_SIO2             (6)
#define WAVESHARE_LCD_SIO3             (7)
#define WAVESHARE_LCD_RST              (39)

#define WAVESHARE_TP_INT               (11)
#define WAVESHARE_TP_RST               (40)

#define WAVESHARE_LCD_WIDTH            (480)
#define WAVESHARE_LCD_HEIGHT           (480)

#define LVGL_PORT_TICK_PERIOD_MS       (2)
#define LVGL_PORT_TASK_MAX_DELAY_MS    (500)
#define LVGL_PORT_TASK_MIN_DELAY_MS    (2)
#define LVGL_PORT_TASK_STACK_SIZE      (16 * 1024)
#define LVGL_PORT_TASK_PRIORITY        (2)
#define LVGL_PORT_TASK_CORE            (ARDUINO_RUNNING_CORE)

/*
 * Single render-mode switch for board ports:
 * - LV_DISPLAY_RENDER_MODE_PARTIAL saves PSRAM and is usually preferred.
 * - LV_DISPLAY_RENDER_MODE_FULL is safer when a panel needs software rotation;
 *   the Waveshare CO5300 showed partial-refresh artifacts in animated
 *   Squareline screens with rotated partial flushes.
 */
#define LVGL_PORT_RENDER_MODE          LV_DISPLAY_RENDER_MODE_FULL
#define LVGL_PORT_BUFFER_LINES         ((LVGL_PORT_RENDER_MODE == LV_DISPLAY_RENDER_MODE_FULL) ? WAVESHARE_LCD_HEIGHT : 20)

/*
 * The CO5300 driver used by Arduino_GFX does not expose a reliable 90/270
 * hardware rotation path on this panel. Keep the LCD at rotation 0 and rotate
 * both flushed pixels and raw touch coordinates in the LVGL port.
 */
#define WAVESHARE_DISPLAY_ROTATION     LV_DISPLAY_ROTATION_270

#ifdef __cplusplus
extern "C" {
#endif

bool lvgl_port_waveshare_init(void);
bool lvgl_port_lock(int timeout_ms);
bool lvgl_port_unlock(void);
lv_display_t *lvgl_port_get_display(void);
lv_indev_t *lvgl_port_get_touch(void);

#ifdef __cplusplus
}
#endif

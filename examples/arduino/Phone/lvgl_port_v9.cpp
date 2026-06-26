/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "lvgl_port_v9.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "lvgl_port";
static SemaphoreHandle_t lvgl_mux = nullptr;
static TaskHandle_t lvgl_task_handle = nullptr;
static esp_timer_handle_t lvgl_tick_timer = nullptr;
static void *lvgl_buf[LVGL_PORT_BUFFER_NUM] = {};
static lv_display_t *lvgl_display = nullptr;
static lv_indev_t *lvgl_touch = nullptr;
static bool lcd_flush_is_async = false;

static void flush_callback(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    ESP_PanelLcd *lcd = static_cast<ESP_PanelLcd *>(lv_display_get_user_data(display));
    const int32_t offsetx1 = area->x1;
    const int32_t offsetx2 = area->x2;
    const int32_t offsety1 = area->y1;
    const int32_t offsety2 = area->y2;

    lcd->drawBitmap(
        offsetx1, offsety1, offsetx2 - offsetx1 + 1, offsety2 - offsety1 + 1,
        reinterpret_cast<const uint8_t *>(px_map)
    );

    if (!lcd_flush_is_async) {
        lv_display_flush_ready(display);
    }
}

IRAM_ATTR static bool on_draw_bitmap_finish_callback(void *user_data)
{
    lv_display_flush_ready(static_cast<lv_display_t *>(user_data));
    return false;
}

static lv_display_t *display_init(ESP_PanelLcd *lcd)
{
    if ((lcd == nullptr) || (lcd->getHandle() == nullptr)) {
        ESP_LOGE(TAG, "Invalid LCD device");
        return nullptr;
    }

    const uint32_t buffer_pixels = LVGL_PORT_BUFFER_SIZE;
    const uint32_t buffer_bytes = buffer_pixels * lv_color_format_get_size(LV_COLOR_FORMAT_NATIVE);

    for (int i = 0; i < LVGL_PORT_BUFFER_NUM; i++) {
        lvgl_buf[i] = heap_caps_malloc(buffer_bytes, LVGL_PORT_BUFFER_MALLOC_CAPS);
        if (lvgl_buf[i] == nullptr) {
            ESP_LOGE(TAG, "Allocate LVGL buffer %d failed", i);
            return nullptr;
        }
    }

    lv_display_t *display = lv_display_create(LVGL_PORT_DISP_WIDTH, LVGL_PORT_DISP_HEIGHT);
    if (display == nullptr) {
        ESP_LOGE(TAG, "Create LVGL display failed");
        return nullptr;
    }

    lv_display_set_user_data(display, lcd);
    lv_display_set_flush_cb(display, flush_callback);
    lv_display_set_buffers(
        display, lvgl_buf[0], lvgl_buf[1], buffer_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    lcd_flush_is_async = lcd->getBus()->getType() != ESP_PANEL_BUS_TYPE_RGB;
    if (lcd_flush_is_async) {
        lcd->attachDrawBitmapFinishCallback(on_draw_bitmap_finish_callback, display);
    }

    return display;
}

static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    ESP_PanelTouch *tp = static_cast<ESP_PanelTouch *>(lv_indev_get_user_data(indev));
    ESP_PanelTouchPoint point;

    const int read_touch_result = tp->readPoints(&point, 1);
    if (read_touch_result > 0) {
        data->point.x = point.x;
        data->point.y = point.y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static lv_indev_t *indev_init(ESP_PanelTouch *tp)
{
    if ((tp == nullptr) || (tp->getHandle() == nullptr)) {
        ESP_LOGE(TAG, "Invalid touch device");
        return nullptr;
    }

    lv_indev_t *indev = lv_indev_create();
    if (indev == nullptr) {
        ESP_LOGE(TAG, "Create LVGL input device failed");
        return nullptr;
    }

    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touchpad_read);
    lv_indev_set_user_data(indev, tp);
    lv_indev_set_display(indev, lvgl_display);

    return indev;
}

#if !LV_TICK_CUSTOM
static void tick_increment(void *arg)
{
    lv_tick_inc(LVGL_PORT_TICK_PERIOD_MS);
}

static bool tick_init(void)
{
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &tick_increment,
        .name = "LVGL tick",
    };

    if (esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Create LVGL tick timer failed");
        return false;
    }

    if (esp_timer_start_periodic(lvgl_tick_timer, LVGL_PORT_TICK_PERIOD_MS * 1000) != ESP_OK) {
        ESP_LOGE(TAG, "Start LVGL tick timer failed");
        return false;
    }

    return true;
}
#endif

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");

    uint32_t task_delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
    while (true) {
        if (lvgl_port_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            lvgl_port_unlock();
        }
        if (task_delay_ms > LVGL_PORT_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_PORT_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_PORT_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

bool lvgl_port_init(ESP_PanelLcd *lcd, ESP_PanelTouch *tp)
{
    if (lcd == nullptr) {
        ESP_LOGE(TAG, "Invalid LCD device");
        return false;
    }

    lv_init();

#if !LV_TICK_CUSTOM
    if (!tick_init()) {
        return false;
    }
#endif

    lvgl_display = display_init(lcd);
    if (lvgl_display == nullptr) {
        return false;
    }

    if (tp != nullptr) {
        lvgl_touch = indev_init(tp);
        if (lvgl_touch == nullptr) {
            return false;
        }
    }

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    if (lvgl_mux == nullptr) {
        ESP_LOGE(TAG, "Create LVGL mutex failed");
        return false;
    }

    const BaseType_t core_id = (LVGL_PORT_TASK_CORE < 0) ? tskNO_AFFINITY : LVGL_PORT_TASK_CORE;
    const BaseType_t ret = xTaskCreatePinnedToCore(
        lvgl_port_task, "lvgl", LVGL_PORT_TASK_STACK_SIZE, nullptr,
        LVGL_PORT_TASK_PRIORITY, &lvgl_task_handle, core_id
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Create LVGL task failed");
        return false;
    }

    return true;
}

bool lvgl_port_lock(int timeout_ms)
{
    if (lvgl_mux == nullptr) {
        return false;
    }

    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

bool lvgl_port_unlock(void)
{
    if (lvgl_mux == nullptr) {
        return false;
    }

    xSemaphoreGiveRecursive(lvgl_mux);
    return true;
}

lv_display_t *lvgl_port_get_display(void)
{
    return lvgl_display;
}

lv_indev_t *lvgl_port_get_touch(void)
{
    return lvgl_touch;
}

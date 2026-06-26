/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "lvgl_port_waveshare.h"

#include <Arduino_GFX_Library.h>
#include <SensorLib.h>
#include <TouchDrv.hpp>
#include <Wire.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "waveshare_port";
static constexpr uint32_t BUFFER_PIXELS = WAVESHARE_LCD_WIDTH * LVGL_PORT_BUFFER_LINES;
static constexpr uint32_t BUFFER_BYTES = BUFFER_PIXELS * 2;
static constexpr uint8_t CST9220_ACK = 0xAB;

static Arduino_DataBus *lcd_bus = nullptr;
static Arduino_CO5300 *lcd = nullptr;
static TouchDrvCST92xx touch;
static bool touch_ok = false;
static volatile bool touch_irq_seen = false;

static SemaphoreHandle_t lvgl_mux = nullptr;
static TaskHandle_t lvgl_task_handle = nullptr;
static esp_timer_handle_t lvgl_tick_timer = nullptr;
static void *lvgl_buf[2] = {};
static void *lvgl_rotation_buf = nullptr;
static lv_display_t *lvgl_display = nullptr;
static lv_indev_t *lvgl_touch = nullptr;

static void IRAM_ATTR touch_irq_cb()
{
    touch_irq_seen = true;
}

static bool lcd_init()
{
    lcd_bus = new Arduino_ESP32QSPI(
        WAVESHARE_LCD_CS,
        WAVESHARE_LCD_SCLK,
        WAVESHARE_LCD_SIO0,
        WAVESHARE_LCD_SIO1,
        WAVESHARE_LCD_SIO2,
        WAVESHARE_LCD_SIO3
    );
    if (lcd_bus == nullptr) {
        ESP_LOGE(TAG, "Create QSPI bus failed");
        return false;
    }

    lcd = new Arduino_CO5300(
        lcd_bus,
        WAVESHARE_LCD_RST,
        0,
        WAVESHARE_LCD_WIDTH,
        WAVESHARE_LCD_HEIGHT,
        0,
        0,
        0,
        0
    );
    if (lcd == nullptr) {
        ESP_LOGE(TAG, "Create CO5300 driver failed");
        return false;
    }

    if (!lcd->begin()) {
        ESP_LOGE(TAG, "CO5300 begin failed");
        return false;
    }

    lcd->setRotation(0);
    lcd->displayOn();
    lcd->setBrightness(200);
    lcd->fillScreen(0x0000);
    ESP_LOGI(TAG, "CO5300 OK %dx%d", WAVESHARE_LCD_WIDTH, WAVESHARE_LCD_HEIGHT);
    return true;
}

static bool touch_init()
{
    Wire.begin(WAVESHARE_I2C_SDA, WAVESHARE_I2C_SCL);

    pinMode(WAVESHARE_TP_INT, INPUT_PULLUP);
    pinMode(WAVESHARE_TP_RST, OUTPUT);
    digitalWrite(WAVESHARE_TP_RST, LOW);
    delay(30);
    digitalWrite(WAVESHARE_TP_RST, HIGH);
    delay(150);

    touch.setPins(WAVESHARE_TP_RST, WAVESHARE_TP_INT);
    touch_ok = touch.begin(Wire, CST92XX_SLAVE_ADDRESS, -1, -1);
    if (!touch_ok) {
        ESP_LOGE(TAG, "CST9220 init failed");
        return false;
    }

    touch.setMaxCoordinates(WAVESHARE_LCD_WIDTH, WAVESHARE_LCD_HEIGHT);
    touch.setSwapXY(false);
    touch.setMirrorXY(false, false);
    attachInterrupt(digitalPinToInterrupt(WAVESHARE_TP_INT), touch_irq_cb, FALLING);

    ESP_LOGI(TAG, "CST9220 OK model=%s", touch.getModelName());
    return true;
}

static bool cst9220_write_ack()
{
    Wire.beginTransmission(CST92XX_SLAVE_ADDRESS);
    Wire.write(0xD0);
    Wire.write(0x00);
    Wire.write(CST9220_ACK);
    return Wire.endTransmission() == 0;
}

static bool cst9220_read_raw_point(uint16_t *x, uint16_t *y)
{
    static constexpr uint8_t CST92XX_TOUCH_BUFFER_SIZE = 15;
    static constexpr uint8_t CST92XX_MAX_TOUCH_POINTS = 2;
    uint8_t buffer[CST92XX_TOUCH_BUFFER_SIZE] = {};

    Wire.beginTransmission(CST92XX_SLAVE_ADDRESS);
    Wire.write(0xD0);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(CST92XX_SLAVE_ADDRESS, CST92XX_TOUCH_BUFFER_SIZE) != CST92XX_TOUCH_BUFFER_SIZE) {
        cst9220_write_ack();
        return false;
    }

    for (uint8_t i = 0; i < CST92XX_TOUCH_BUFFER_SIZE; i++) {
        buffer[i] = Wire.read();
    }
    cst9220_write_ack();

    if (buffer[0] == CST9220_ACK || buffer[6] != CST9220_ACK) {
        return false;
    }

    const uint8_t num_points = buffer[5] & 0x7F;
    if (num_points == 0 || num_points > CST92XX_MAX_TOUCH_POINTS) {
        return false;
    }

    for (uint8_t i = 0; i < num_points; i++) {
        const uint8_t *point_data = buffer + (i * 5) + (i == 0 ? 0 : 2);
        const uint8_t id = point_data[0] >> 4;
        const uint8_t event = point_data[0] & 0x0F;
        if (event != 0x06 || id >= CST92XX_MAX_TOUCH_POINTS) {
            continue;
        }

        *x = (static_cast<uint16_t>(point_data[1]) << 4) | (point_data[3] >> 4);
        *y = (static_cast<uint16_t>(point_data[2]) << 4) | (point_data[3] & 0x0F);
        return true;
    }

    return false;
}

static void rotate_touch_point(uint16_t raw_x, uint16_t raw_y, lv_point_t *point)
{
    raw_x = constrain(raw_x, 0, WAVESHARE_LCD_WIDTH - 1);
    raw_y = constrain(raw_y, 0, WAVESHARE_LCD_HEIGHT - 1);

    switch (WAVESHARE_DISPLAY_ROTATION) {
    case LV_DISPLAY_ROTATION_90:
        point->x = raw_y;
        point->y = WAVESHARE_LCD_WIDTH - 1 - raw_x;
        break;
    case LV_DISPLAY_ROTATION_180:
        point->x = WAVESHARE_LCD_WIDTH - 1 - raw_x;
        point->y = WAVESHARE_LCD_HEIGHT - 1 - raw_y;
        break;
    case LV_DISPLAY_ROTATION_270:
        point->x = WAVESHARE_LCD_HEIGHT - 1 - raw_y;
        point->y = raw_x;
        break;
    case LV_DISPLAY_ROTATION_0:
    default:
        point->x = raw_x;
        point->y = raw_y;
        break;
    }
}

static void flush_rotated(const lv_area_t *area, const uint16_t *src)
{
    const int32_t src_w = area->x2 - area->x1 + 1;
    const int32_t src_h = area->y2 - area->y1 + 1;
    uint16_t *dst = reinterpret_cast<uint16_t *>(lvgl_rotation_buf);

    switch (WAVESHARE_DISPLAY_ROTATION) {
    case LV_DISPLAY_ROTATION_90:
        for (int32_t y = 0; y < src_h; y++) {
            for (int32_t x = 0; x < src_w; x++) {
                dst[(x * src_h) + (src_h - 1 - y)] = src[(y * src_w) + x];
            }
        }
        lcd->draw16bitRGBBitmap(
            WAVESHARE_LCD_HEIGHT - 1 - area->y2,
            area->x1,
            dst,
            src_h,
            src_w
        );
        break;

    case LV_DISPLAY_ROTATION_180:
        for (int32_t y = 0; y < src_h; y++) {
            for (int32_t x = 0; x < src_w; x++) {
                dst[((src_h - 1 - y) * src_w) + (src_w - 1 - x)] = src[(y * src_w) + x];
            }
        }
        lcd->draw16bitRGBBitmap(
            WAVESHARE_LCD_WIDTH - 1 - area->x2,
            WAVESHARE_LCD_HEIGHT - 1 - area->y2,
            dst,
            src_w,
            src_h
        );
        break;

    case LV_DISPLAY_ROTATION_270:
        for (int32_t y = 0; y < src_h; y++) {
            for (int32_t x = 0; x < src_w; x++) {
                dst[((src_w - 1 - x) * src_h) + y] = src[(y * src_w) + x];
            }
        }
        lcd->draw16bitRGBBitmap(
            area->y1,
            WAVESHARE_LCD_WIDTH - 1 - area->x2,
            dst,
            src_h,
            src_w
        );
        break;

    case LV_DISPLAY_ROTATION_0:
    default:
        lcd->draw16bitRGBBitmap(area->x1, area->y1, src, src_w, src_h);
        break;
    }
}

static void flush_callback(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    if (lcd != nullptr) {
        const uint16_t *src = reinterpret_cast<uint16_t *>(px_map);
        if (WAVESHARE_DISPLAY_ROTATION == LV_DISPLAY_ROTATION_0 || lvgl_rotation_buf == nullptr) {
            lcd->draw16bitRGBBitmap(
                area->x1,
                area->y1,
                src,
                area->x2 - area->x1 + 1,
                area->y2 - area->y1 + 1
            );
        } else {
            flush_rotated(area, src);
        }
    }
    lv_display_flush_ready(display);
}

static lv_display_t *display_init()
{
    for (int i = 0; i < 2; i++) {
        lvgl_buf[i] = heap_caps_malloc(BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (lvgl_buf[i] == nullptr) {
            lvgl_buf[i] = heap_caps_malloc(BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (lvgl_buf[i] == nullptr) {
            ESP_LOGE(TAG, "Allocate LVGL buffer %d failed", i);
            return nullptr;
        }
    }

    if (WAVESHARE_DISPLAY_ROTATION != LV_DISPLAY_ROTATION_0) {
        lvgl_rotation_buf = heap_caps_malloc(BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (lvgl_rotation_buf == nullptr) {
            lvgl_rotation_buf = heap_caps_malloc(BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (lvgl_rotation_buf == nullptr) {
            ESP_LOGE(TAG, "Allocate LVGL rotation buffer failed");
            return nullptr;
        }
    }

    lv_display_t *display = lv_display_create(WAVESHARE_LCD_WIDTH, WAVESHARE_LCD_HEIGHT);
    if (display == nullptr) {
        ESP_LOGE(TAG, "Create LVGL display failed");
        return nullptr;
    }

    lv_display_set_flush_cb(display, flush_callback);
    lv_display_set_buffers(display, lvgl_buf[0], lvgl_buf[1], BUFFER_BYTES, LVGL_PORT_RENDER_MODE);
    return display;
}

static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    if (!touch_ok) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    const bool irq_active = digitalRead(WAVESHARE_TP_INT) == LOW;
    if (!touch_irq_seen && !irq_active) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    touch_irq_seen = false;

    uint16_t raw_x = 0;
    uint16_t raw_y = 0;
    if (cst9220_read_raw_point(&raw_x, &raw_y)) {
        rotate_touch_point(raw_x, raw_y, &data->point);
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static lv_indev_t *indev_init()
{
    lv_indev_t *indev = lv_indev_create();
    if (indev == nullptr) {
        ESP_LOGE(TAG, "Create LVGL input device failed");
        return nullptr;
    }

    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touchpad_read);
    lv_indev_set_display(indev, lvgl_display);
    return indev;
}

#if !LV_TICK_CUSTOM
static void tick_increment(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_PORT_TICK_PERIOD_MS);
}

static bool tick_init()
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
    (void)arg;
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

bool lvgl_port_waveshare_init(void)
{
    if (!lcd_init()) {
        return false;
    }
    if (!touch_init()) {
        return false;
    }

    lv_init();

#if !LV_TICK_CUSTOM
    if (!tick_init()) {
        return false;
    }
#endif

    lvgl_display = display_init();
    if (lvgl_display == nullptr) {
        return false;
    }

    lvgl_touch = indev_init();
    if (lvgl_touch == nullptr) {
        return false;
    }

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    if (lvgl_mux == nullptr) {
        ESP_LOGE(TAG, "Create LVGL mutex failed");
        return false;
    }

    const BaseType_t core_id = (LVGL_PORT_TASK_CORE < 0) ? tskNO_AFFINITY : LVGL_PORT_TASK_CORE;
    const BaseType_t ret = xTaskCreatePinnedToCore(
        lvgl_port_task,
        "lvgl",
        LVGL_PORT_TASK_STACK_SIZE,
        nullptr,
        LVGL_PORT_TASK_PRIORITY,
        &lvgl_task_handle,
        core_id
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

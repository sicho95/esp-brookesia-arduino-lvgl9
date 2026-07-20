/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 * SPDX-License-Identifier: CC0-1.0
 */
#include "waveshare_system.hpp"

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <lvgl.h>
#include <esp_brookesia.hpp>
#include "lvgl_port_waveshare.h"

#if __has_include(<XPowersLib.h>)
#include <XPowersLib.h>
#define WAVESHARE_SYSTEM_HAS_PMU 1
#else
#define WAVESHARE_SYSTEM_HAS_PMU 0
#endif

#if __has_include(<SensorQMI8658.hpp>)
#include <SensorQMI8658.hpp>
#define WAVESHARE_SYSTEM_HAS_IMU 1
#else
#define WAVESHARE_SYSTEM_HAS_IMU 0
#endif

namespace {

constexpr uint32_t DISPLAY_OFF_DELAY_MS = 60UL * 1000UL;
constexpr uint32_t CONNECTED_STANDBY_DELAY_MS = 10UL * 60UL * 1000UL;
constexpr uint32_t DEEP_SLEEP_DELAY_MS = 20UL * 60UL * 1000UL;
constexpr uint32_t NOTIFICATION_WAKE_DELAY_MS = 30UL * 1000UL;
constexpr uint32_t NOTIFICATION_BANNER_DELAY_MS = 12UL * 1000UL;
constexpr uint32_t SYSTEM_TICK_PERIOD_MS = 20;
constexpr uint32_t PMU_THERMAL_POLL_PERIOD_MS = 5000;
constexpr uint32_t PMU_IRQ_POLL_PERIOD_MS = 100;
constexpr uint32_t IMU_POLL_PERIOD_MS = 100;
constexpr uint32_t NTP_RETRY_PERIOD_MS = 60UL * 1000UL;
constexpr uint8_t MAX_NOTIFICATIONS = 6;
constexpr uint8_t MAX_SCHEDULED_JOBS = 8;
constexpr int PIN_KEY_PLUS = 18;
constexpr int PIN_KEY_MINUS = 0;
constexpr int PIN_IMU_INT1 = 17;
constexpr int PIN_IMU_INT2 = 21;
constexpr uint8_t QMI8658_I2C_ADDRESS = 0x6B;
constexpr uint32_t LONG_PRESS_MS = 750;

/* The AXP2101 sensor measures PMU die temperature, not the cell. These
 * conservative limits protect the charging circuit until an optional battery
 * NTC is wired and exposed by the board. */
constexpr float PMU_CHARGE_REDUCE_C = 70.0F;
constexpr float PMU_CHARGE_STOP_C = 80.0F;
constexpr float PMU_CHARGE_RESUME_C = 65.0F;

struct BatteryProfile {
    uint16_t capacity_mah;
    uint8_t charge_voltage;  // AXP2101 enum, never a free-form voltage.
    uint8_t charge_current;  // AXP2101 enum, bounded by the profile.
    bool battery_care;
};

struct Settings {
    uint8_t brightness;
    uint32_t display_off_delay_ms;
    bool wifi_enabled;
    bool ble_enabled;
    bool airplane_mode;
    bool rotation_locked;
    BatteryProfile battery;
};

struct NotificationEntry {
    bool used;
    bool unread;
    int app_id;
    uint32_t created_ms;
    char title[32];
    char message[80];
};

struct ScheduledJob {
    bool used;
    volatile bool running;
    int app_id;
    uint32_t period_ms;
    uint32_t next_run_ms;
    WaveshareScheduledJobCallback callback;
    void *context;
};

ESP_Brookesia_Phone *phone = nullptr;
Preferences preferences;
Settings settings = {
    .brightness = 78,
    .display_off_delay_ms = DISPLAY_OFF_DELAY_MS,
    .wifi_enabled = true,
    .ble_enabled = true,
    .airplane_mode = false,
    .rotation_locked = false,
    .battery = {
        .capacity_mah = 1000,
        .charge_voltage = 3, // AXP2101 4.2 V
        .charge_current = 10, // AXP2101 300 mA
        .battery_care = false,
    },
};

WaveshareSystemPowerState power_state = WAVESHARE_SYSTEM_ACTIVE;
uint32_t last_user_activity_ms = 0;
uint32_t last_tick_ms = 0;
uint32_t last_thermal_poll_ms = 0;
uint32_t last_pmu_irq_poll_ms = 0;
uint32_t last_pwr_action_ms = 0;
bool pwr_action_seen = false;
uint32_t last_imu_poll_ms = 0;
uint32_t last_ntp_attempt_ms = 0;
uint8_t display_lease_count = 0;
bool charge_paused_for_temperature = false;
bool charge_reduced_for_temperature = false;
bool control_center_visible = false;
bool key_plus_was_down = false;
bool key_minus_was_down = false;
volatile bool notification_ui_dirty = false;
volatile bool notification_wake_pending = false;
volatile bool notification_banner_pending = false;
bool ntp_started = false;
bool time_synchronized = false;
bool notification_wake_active = false;
uint32_t notification_wake_deadline_ms = 0;
uint32_t key_plus_down_ms = 0;
uint32_t key_minus_down_ms = 0;
lv_obj_t *control_center = nullptr;
lv_obj_t *control_page = nullptr;
lv_obj_t *notification_page = nullptr;
lv_obj_t *state_label = nullptr;
lv_obj_t *battery_label = nullptr;
lv_obj_t *wifi_button_label = nullptr;
lv_obj_t *airplane_button_label = nullptr;
lv_obj_t *rotation_button_label = nullptr;
lv_obj_t *ble_button_label = nullptr;
lv_obj_t *brightness_slider = nullptr;
lv_obj_t *wifi_button = nullptr;
lv_obj_t *airplane_button = nullptr;
lv_obj_t *rotation_button = nullptr;
lv_obj_t *ble_button = nullptr;
lv_obj_t *notification_list = nullptr;
lv_obj_t *time_label = nullptr;
lv_obj_t *control_page_dot = nullptr;
lv_obj_t *notification_page_dot = nullptr;
lv_obj_t *notification_badge = nullptr;
lv_obj_t *notification_badge_label = nullptr;
lv_obj_t *fullscreen_notification_badge = nullptr;
lv_obj_t *fullscreen_notification_badge_label = nullptr;
lv_obj_t *status_radio_icons = nullptr;
lv_obj_t *status_charge_icon = nullptr;
lv_obj_t *notification_banner = nullptr;
lv_obj_t *notification_banner_title = nullptr;
lv_obj_t *notification_banner_message = nullptr;
uint8_t control_center_page_index = 0;
bool control_center_touch_active = false;
lv_point_t control_center_touch_start = {};
int notification_banner_app_id = -1;
uint32_t notification_banner_deadline_ms = 0;
NotificationEntry pending_banner_notification = {};
NotificationEntry notifications[MAX_NOTIFICATIONS] = {};
ScheduledJob scheduled_jobs[MAX_SCHEDULED_JOBS] = {};
portMUX_TYPE service_mux = portMUX_INITIALIZER_UNLOCKED;

void set_power_state(WaveshareSystemPowerState next);
void show_control_center_page(uint8_t page);
void refresh_status_indicators();

#if WAVESHARE_SYSTEM_HAS_PMU
XPowersAXP2101 *pmu = nullptr;
#endif

#if WAVESHARE_SYSTEM_HAS_IMU
SensorQMI8658 *imu = nullptr;
bool imu_ok = false;
int imu_orientation = 0;
int imu_candidate_orientation = -1;
uint8_t imu_candidate_samples = 0;
#endif

void notification_click_cb(lv_event_t *event)
{
    const int app_id = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    waveshare_system_clear_notifications(app_id);
    waveshare_system_hide_control_center();
    if (app_id >= 0 && phone != nullptr) {
        ESP_Brookesia_CoreAppEventData_t app_event = {};
        app_event.id = app_id;
        app_event.type = ESP_BROOKESIA_CORE_APP_EVENT_TYPE_START;
        phone->sendAppEvent(&app_event);
    }
}

void notification_delete_cb(lv_event_t *event)
{
    const int slot = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(event)));
    if (slot < 0 || slot >= MAX_NOTIFICATIONS) return;
    portENTER_CRITICAL(&service_mux);
    notifications[slot].used = false;
    notification_ui_dirty = true;
    portEXIT_CRITICAL(&service_mux);
}

void clear_all_notifications_cb(lv_event_t *)
{
    portENTER_CRITICAL(&service_mux);
    for (uint8_t i = 0; i < MAX_NOTIFICATIONS; i++) notifications[i].used = false;
    notification_ui_dirty = true;
    portEXIT_CRITICAL(&service_mux);
}

void notification_badge_cb(lv_event_t *)
{
    waveshare_system_show_control_center();
    show_control_center_page(1);
}

void notification_banner_cb(lv_event_t *)
{
    const int app_id = notification_banner_app_id;
    lv_obj_add_flag(notification_banner, LV_OBJ_FLAG_HIDDEN);
    notification_banner_deadline_ms = 0;
    waveshare_system_clear_notifications(app_id);
    if (app_id >= 0 && phone != nullptr) {
        ESP_Brookesia_CoreAppEventData_t app_event = {};
        app_event.id = app_id;
        app_event.type = ESP_BROOKESIA_CORE_APP_EVENT_TYPE_START;
        phone->sendAppEvent(&app_event);
    }
}

void refresh_notification_list()
{
    if (notification_list == nullptr || !notification_ui_dirty) return;
    notification_ui_dirty = false;
    lv_obj_clean(notification_list);

    NotificationEntry snapshot[MAX_NOTIFICATIONS] = {};
    portENTER_CRITICAL(&service_mux);
    memcpy(snapshot, notifications, sizeof(snapshot));
    portEXIT_CRITICAL(&service_mux);

    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_NOTIFICATIONS; i++) {
        if (!snapshot[i].used) continue;
        count++;
        lv_obj_t *button = lv_button_create(notification_list);
        lv_obj_set_width(button, LV_PCT(100));
        lv_obj_set_height(button, LV_SIZE_CONTENT);
        lv_obj_set_style_radius(button, 6, 0);
        lv_obj_set_style_bg_color(button, lv_color_hex(snapshot[i].unread ? 0x263F52 : 0x253039), 0);
        lv_obj_set_flex_flow(button, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(button, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_add_event_cb(button, notification_click_cb, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(snapshot[i].app_id)));
        lv_obj_t *label = lv_label_create(button);
        lv_label_set_text_fmt(label, "%s\n%s", snapshot[i].title, snapshot[i].message);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_flex_grow(label, 1);
        lv_obj_t *remove = lv_button_create(button);
        lv_obj_set_size(remove, 42, 42);
        lv_obj_set_style_radius(remove, 8, 0);
        lv_obj_set_style_bg_color(remove, lv_color_hex(0x394650), 0);
        lv_obj_add_event_cb(remove, notification_delete_cb, LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_t *remove_label = lv_label_create(remove);
        lv_label_set_text(remove_label, "X");
        lv_obj_center(remove_label);
    }
    if (count == 0) {
        lv_obj_t *empty = lv_label_create(notification_list);
        lv_label_set_text(empty, "Aucune notification");
        lv_obj_set_style_text_color(empty, lv_color_hex(0x8FA2B0), 0);
    }
}

void scheduled_job_task(void *parameter)
{
    ScheduledJob *job = static_cast<ScheduledJob *>(parameter);
    if (job != nullptr && job->callback != nullptr) job->callback(job->context);
    if (job != nullptr) job->running = false;
    vTaskDelete(nullptr);
}

void dispatch_scheduled_jobs(uint32_t now)
{
    for (uint8_t i = 0; i < MAX_SCHEDULED_JOBS; i++) {
        ScheduledJob &job = scheduled_jobs[i];
        if (!job.used || job.running || static_cast<int32_t>(now - job.next_run_ms) < 0) continue;
        job.running = true;
        job.next_run_ms = now + job.period_ms;
        if (xTaskCreatePinnedToCore(scheduled_job_task, "brookesia_job", 4096, &job, 1, nullptr, 0) != pdPASS) {
            job.running = false;
            Serial.printf("[system] scheduler: app %d task creation failed\n", job.app_id);
        }
    }
}

void update_network_time(uint32_t now)
{
    if (settings.airplane_mode || !settings.wifi_enabled || WiFi.status() != WL_CONNECTED) return;
    if (!ntp_started || now - last_ntp_attempt_ms >= NTP_RETRY_PERIOD_MS) {
        configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "pool.ntp.org", "time.google.com");
        ntp_started = true;
        last_ntp_attempt_ms = now;
        Serial.println("[system] NTP sync requested (Europe/Paris)");
    }
    time_t current_time = time(nullptr);
    time_synchronized = current_time >= 1704067200; // 2024-01-01
}

const char *power_state_name(WaveshareSystemPowerState state)
{
    switch (state) {
    case WAVESHARE_SYSTEM_ACTIVE: return "Actif";
    case WAVESHARE_SYSTEM_SCREEN_OFF: return "Ecran eteint";
    case WAVESHARE_SYSTEM_CONNECTED_STANDBY: return "Veille connectee";
    case WAVESHARE_SYSTEM_DEEP_SLEEP_PENDING: return "Deep sleep pret";
    default: return "Inconnu";
    }
}

void save_settings()
{
    preferences.putUChar("brightness", settings.brightness);
    preferences.putULong("screen_ms", settings.display_off_delay_ms);
    preferences.putBool("wifi", settings.wifi_enabled);
    preferences.putBool("ble", settings.ble_enabled);
    preferences.putBool("airplane", settings.airplane_mode);
    preferences.putBool("rot_lock", settings.rotation_locked);
    preferences.putUShort("bat_mah", settings.battery.capacity_mah);
    preferences.putUChar("bat_v", settings.battery.charge_voltage);
    preferences.putUChar("bat_i", settings.battery.charge_current);
    preferences.putBool("bat_care", settings.battery.battery_care);
}

void load_settings()
{
    preferences.begin("brookesia", false);
    settings.brightness = preferences.getUChar("brightness", settings.brightness);
    settings.display_off_delay_ms = preferences.getULong("screen_ms", settings.display_off_delay_ms);
    settings.wifi_enabled = preferences.getBool("wifi", settings.wifi_enabled);
    settings.ble_enabled = preferences.getBool("ble", settings.ble_enabled);
    settings.airplane_mode = preferences.getBool("airplane", settings.airplane_mode);
    settings.rotation_locked = preferences.getBool("rot_lock", settings.rotation_locked);
    settings.battery.capacity_mah = preferences.getUShort("bat_mah", settings.battery.capacity_mah);
    settings.battery.charge_voltage = preferences.getUChar("bat_v", settings.battery.charge_voltage);
    settings.battery.charge_current = preferences.getUChar("bat_i", settings.battery.charge_current);
    settings.battery.battery_care = preferences.getBool("bat_care", settings.battery.battery_care);

    settings.brightness = constrain(settings.brightness, 1, 100);
    settings.display_off_delay_ms = constrain(settings.display_off_delay_ms, 15UL * 1000UL, 30UL * 60UL * 1000UL);
    settings.battery.capacity_mah = constrain(settings.battery.capacity_mah, 100, 10000);
    /* Standard 1S Li-Po must never select LiHV voltages. */
    settings.battery.charge_voltage = settings.battery.battery_care ? 2 : 3;
    if (settings.battery.charge_current > 10) {
        settings.battery.charge_current = 10;
    }
}

void apply_radio_settings()
{
    if (settings.airplane_mode || !settings.wifi_enabled) {
        WiFi.mode(WIFI_OFF);
    } else {
        WiFi.mode(WIFI_STA);
    }
    /* BLE is a permission owned by this service, not a bare controller state.
     * NimBLE/Bluedroid transports must start and stop their host stack cleanly;
     * calling btStartMode() without a host crashes Arduino-ESP32 3.3.8. */
}

#if WAVESHARE_SYSTEM_HAS_PMU
void apply_battery_profile()
{
    if (pmu == nullptr) {
        return;
    }
    pmu->disableCellbatteryCharge();
    pmu->setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    pmu->setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_50MA);
    pmu->enableChargerTerminationLimit();
    pmu->setChargerConstantCurr(settings.battery.charge_current);
    pmu->setChargeTargetVoltage(settings.battery.charge_voltage);
    pmu->enableCellbatteryCharge();
}

bool init_pmu()
{
    pmu = new XPowersAXP2101();
    if (pmu == nullptr || !pmu->begin(Wire, AXP2101_SLAVE_ADDRESS, WAVESHARE_I2C_SDA, WAVESHARE_I2C_SCL)) {
        delete pmu;
        pmu = nullptr;
        Serial.println("[system] AXP2101 unavailable");
        return false;
    }
    pmu->enableBattDetection();
    pmu->enableBattVoltageMeasure();
    pmu->enableVbusVoltageMeasure();
    pmu->enableSystemVoltageMeasure();
    pmu->enableTemperatureMeasure();
    pmu->enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ);
    pmu->clearIrqStatus();
    pmu->setThermaThreshold(XPOWERS_AXP2101_THREMAL_80DEG);
    apply_battery_profile();
    Serial.println("[system] AXP2101 battery profile active");
    return true;
}

void poll_pmu_power_key(uint32_t now)
{
    if (pmu == nullptr || now - last_pmu_irq_poll_ms < PMU_IRQ_POLL_PERIOD_MS) {
        return;
    }
    last_pmu_irq_poll_ms = now;
    pmu->getIrqStatus();
    const bool short_press = pmu->isPekeyShortPressIrq();
    const bool long_press = pmu->isPekeyLongPressIrq();
    if (short_press || long_press) {
        pmu->clearIrqStatus();
        if (pwr_action_seen && now - last_pwr_action_ms < 1200) return;
        pwr_action_seen = true;
        last_pwr_action_ms = now;
        if (long_press) {
            Serial.println("[system] PWR long press -> orderly PMU shutdown");
            save_settings();
            lvgl_port_set_display_power(false);
            delay(80);
            pmu->shutdown();
            return;
        }
        if (power_state == WAVESHARE_SYSTEM_ACTIVE) {
            set_power_state(WAVESHARE_SYSTEM_CONNECTED_STANDBY);
            Serial.println("[system] PWR short press -> connected standby");
        } else {
            waveshare_system_note_user_activity();
            Serial.println("[system] PWR short press -> active");
        }
    }
}

void update_pmu_thermal_policy(uint32_t now)
{
    if (pmu == nullptr || now - last_thermal_poll_ms < PMU_THERMAL_POLL_PERIOD_MS) {
        return;
    }
    last_thermal_poll_ms = now;
    const float temperature_c = pmu->getTemperature();
    if (isnan(temperature_c)) {
        return;
    }

    if (temperature_c >= PMU_CHARGE_STOP_C) {
        if (!charge_paused_for_temperature) {
            pmu->disableCellbatteryCharge();
            charge_paused_for_temperature = true;
            charge_reduced_for_temperature = false;
            Serial.printf("[system] charge paused: PMU %.1f C\n", temperature_c);
        }
        return;
    }
    if (charge_paused_for_temperature) {
        if (temperature_c <= PMU_CHARGE_RESUME_C) {
            charge_paused_for_temperature = false;
            apply_battery_profile();
            Serial.printf("[system] charge resumed: PMU %.1f C\n", temperature_c);
        }
        return;
    }
    if (temperature_c >= PMU_CHARGE_REDUCE_C) {
        if (!charge_reduced_for_temperature) {
            pmu->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_100MA);
            charge_reduced_for_temperature = true;
            Serial.printf("[system] charge reduced: PMU %.1f C\n", temperature_c);
        }
    } else if (charge_reduced_for_temperature && temperature_c <= PMU_CHARGE_RESUME_C) {
        apply_battery_profile();
        charge_reduced_for_temperature = false;
        Serial.printf("[system] charge current restored: PMU %.1f C\n", temperature_c);
    }
}
#endif

#if WAVESHARE_SYSTEM_HAS_IMU
bool init_imu()
{
    imu = new SensorQMI8658();
    if (imu == nullptr || !imu->begin(Wire, QMI8658_I2C_ADDRESS, PIN_IMU_INT1, PIN_IMU_INT2)) {
        delete imu;
        imu = nullptr;
        Serial.println("[system] QMI8658 unavailable; rotation stays fixed");
        return false;
    }
    imu->configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_62_5Hz, SensorQMI8658::LPF_MODE_0);
    imu->enableAccelerometer();
    imu_ok = true;
    Serial.println("[system] QMI8658 auto-rotation ready");
    return true;
}

void poll_auto_rotation(uint32_t now)
{
    if (!imu_ok || imu == nullptr || settings.rotation_locked || now - last_imu_poll_ms < IMU_POLL_PERIOD_MS) {
        return;
    }
    last_imu_poll_ms = now;
    float ax = 0;
    float ay = 0;
    float az = 0;
    if (!imu->getAccelerometer(ax, ay, az)) {
        return;
    }
    const float abs_x = fabsf(ax);
    const float abs_y = fabsf(ay);
    if ((abs_x < 0.45F && abs_y < 0.45F) || fabsf(abs_x - abs_y) < 0.16F) {
        return;
    }
    const int orientation = abs_x > abs_y ? (ax > 0 ? 3 : 1) : (ay < 0 ? 0 : 2);
    if (orientation != imu_candidate_orientation) {
        imu_candidate_orientation = orientation;
        imu_candidate_samples = 1;
        return;
    }
    if (imu_candidate_samples < 4) {
        imu_candidate_samples++;
        return;
    }
    if (orientation == imu_orientation) {
        return;
    }
    imu_orientation = orientation;
    /* The QMI8658 axes are opposite to the logical display axes. The initial
     * portrait orientation remains the known-good 270 degree port rotation. */
    const lv_display_rotation_t rotation =
        orientation == 0 ? LV_DISPLAY_ROTATION_90 :
        orientation == 1 ? LV_DISPLAY_ROTATION_180 :
        orientation == 2 ? LV_DISPLAY_ROTATION_270 : LV_DISPLAY_ROTATION_0;
    lvgl_port_set_rotation(rotation);
    Serial.printf("[system] auto rotation: imu=%d display=%d\n", orientation, rotation);
}
#endif

void refresh_control_center()
{
    refresh_status_indicators();
    if (state_label != nullptr) {
        lv_label_set_text_fmt(state_label, "%s\nVeille ecran: %lus\nLease ecran: %u", power_state_name(power_state),
                              settings.display_off_delay_ms / 1000UL, display_lease_count);
    }
    if (battery_label != nullptr) {
#if WAVESHARE_SYSTEM_HAS_PMU
        if (pmu != nullptr && pmu->isBatteryConnect()) {
            lv_label_set_text_fmt(battery_label, "Batterie %u%%  %.2fV%s\nCharge: %s", (unsigned)pmu->getBatteryPercent(),
                                  pmu->getBattVoltage() / 1000.0F, pmu->isCharging() ? "  charge" : "",
                                  charge_paused_for_temperature ? "pausee (temperature)" :
                                  (charge_reduced_for_temperature ? "ralentie (temperature)" : "normale"));
        } else {
            lv_label_set_text(battery_label, "Batterie/PMU indisponible");
        }
#else
        lv_label_set_text(battery_label, "Installer XPowersLib pour AXP2101");
#endif
    }
    if (wifi_button_label != nullptr) {
        const bool enabled = settings.wifi_enabled && !settings.airplane_mode;
        lv_label_set_text(wifi_button_label, LV_SYMBOL_WIFI);
        lv_obj_set_style_bg_color(wifi_button, lv_color_hex(enabled ? 0x1976D2 : 0x2B3540), 0);
    }
    if (airplane_button_label != nullptr) {
        lv_obj_set_style_bg_color(airplane_button, lv_color_hex(settings.airplane_mode ? 0xE07822 : 0x2B3540), 0);
    }
    if (ble_button_label != nullptr) {
        const bool enabled = settings.ble_enabled && !settings.airplane_mode;
        lv_label_set_text(ble_button_label, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_bg_color(ble_button, lv_color_hex(enabled ? 0x1976D2 : 0x2B3540), 0);
    }
    if (rotation_button_label != nullptr) {
        lv_label_set_text(rotation_button_label, LV_SYMBOL_REFRESH);
        lv_obj_set_style_bg_color(rotation_button, lv_color_hex(settings.rotation_locked ? 0x1976D2 : 0x2B3540), 0);
    }
    if (brightness_slider != nullptr) {
        lv_slider_set_value(brightness_slider, settings.brightness, LV_ANIM_OFF);
    }
    if (time_label != nullptr) {
        lv_label_set_text(time_label, time_synchronized ? "Heure: synchronisee (Europe/Paris)" :
                          (WiFi.status() == WL_CONNECTED ? "Heure: synchronisation..." : "Heure: en attente du Wi-Fi"));
    }
    refresh_notification_list();
}

void refresh_status_indicators()
{
    if (phone == nullptr) return;
    ESP_Brookesia_StatusBar *status_bar = phone->getHome().getStatusBar();
    const bool status_bar_visible = status_bar != nullptr && status_bar->checkVisible();
    const bool wifi_enabled = settings.wifi_enabled && !settings.airplane_mode;
    const bool ble_enabled = settings.ble_enabled && !settings.airplane_mode;
    const uint8_t notification_count = waveshare_system_get_notification_count();

    if (status_bar != nullptr) {
        const bool wifi_connected = wifi_enabled && WiFi.status() == WL_CONNECTED;
        status_bar->setWifiIconState(!wifi_enabled ? -1 : (wifi_connected ? 3 : 0));
        status_bar->setWifiIconColor(wifi_connected ? lv_color_hex(0x38C172) : lv_color_white());
        status_bar->hideBatteryPercent();
#if WAVESHARE_SYSTEM_HAS_PMU
        if (pmu != nullptr && pmu->isBatteryConnect()) {
            status_bar->setBatteryPercent(false, pmu->getBatteryPercent());
            if (status_charge_icon != nullptr) {
                lv_obj_set_flag(status_charge_icon, LV_OBJ_FLAG_HIDDEN, !pmu->isVbusIn());
            }
        } else if (status_charge_icon != nullptr) {
            lv_obj_add_flag(status_charge_icon, LV_OBJ_FLAG_HIDDEN);
        }
#endif
    }
    if (status_radio_icons != nullptr) {
        if (settings.airplane_mode) {
            lv_label_set_text(status_radio_icons, LV_SYMBOL_GPS);
            lv_obj_set_style_text_color(status_radio_icons, lv_color_white(), 0);
        } else if (ble_enabled) {
            lv_label_set_text(status_radio_icons, LV_SYMBOL_BLUETOOTH);
            /* Arduino's BLE host is intentionally not started yet, so this is
             * white while merely enabled. A future host connection callback
             * will change it to blue only after a real peer connection. */
            lv_obj_set_style_text_color(status_radio_icons, lv_color_white(), 0);
        } else {
            lv_label_set_text(status_radio_icons, "");
        }
    }
    if (notification_count == 0) {
        if (notification_badge != nullptr) lv_obj_add_flag(notification_badge, LV_OBJ_FLAG_HIDDEN);
        if (fullscreen_notification_badge != nullptr) lv_obj_add_flag(fullscreen_notification_badge, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (notification_badge != nullptr) {
        lv_label_set_text_fmt(notification_badge_label, LV_SYMBOL_BELL " %u", notification_count);
        lv_obj_set_flag(notification_badge, LV_OBJ_FLAG_HIDDEN, !status_bar_visible);
    }
    if (fullscreen_notification_badge != nullptr) {
        lv_label_set_text_fmt(fullscreen_notification_badge_label, LV_SYMBOL_BELL " %u", notification_count);
        lv_obj_set_flag(fullscreen_notification_badge, LV_OBJ_FLAG_HIDDEN, status_bar_visible);
    }
}

void close_button_cb(lv_event_t *)
{
    waveshare_system_hide_control_center();
}

void wifi_button_cb(lv_event_t *)
{
    settings.wifi_enabled = !settings.wifi_enabled;
    settings.airplane_mode = false;
    apply_radio_settings();
    save_settings();
    refresh_control_center();
}

void airplane_button_cb(lv_event_t *)
{
    settings.airplane_mode = !settings.airplane_mode;
    apply_radio_settings();
    save_settings();
    refresh_control_center();
}

void ble_button_cb(lv_event_t *)
{
    settings.ble_enabled = !settings.ble_enabled;
    settings.airplane_mode = false;
    apply_radio_settings();
    save_settings();
    refresh_control_center();
}

void rotation_button_cb(lv_event_t *)
{
    settings.rotation_locked = !settings.rotation_locked;
    save_settings();
    refresh_control_center();
}

void brightness_cb(lv_event_t *event)
{
    settings.brightness = lv_slider_get_value((lv_obj_t *)lv_event_get_target(event));
    lvgl_port_set_brightness(settings.brightness);
    save_settings();
}

lv_obj_t *add_button(lv_obj_t *parent, const char *text, lv_event_cb_t callback, lv_obj_t **label_out)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 132, 58);
    lv_obj_set_style_radius(button, 8, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x27313B), 0);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    if (label_out != nullptr) {
        *label_out = label;
    }
    return button;
}

lv_obj_t *add_control_tile(lv_obj_t *parent, const char *text, lv_event_cb_t callback, lv_obj_t **label_out)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 100, 82);
    lv_obj_set_style_radius(button, 8, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x2B3540), 0);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    if (label_out != nullptr) *label_out = label;
    return button;
}

void add_airplane_icon(lv_obj_t *button)
{
    lv_obj_t *fuselage = lv_obj_create(button);
    lv_obj_set_size(fuselage, 7, 46);
    lv_obj_center(fuselage);
    lv_obj_set_style_radius(fuselage, 4, 0);
    lv_obj_set_style_bg_color(fuselage, lv_color_white(), 0);
    lv_obj_set_style_border_width(fuselage, 0, 0);

    lv_obj_t *wings = lv_obj_create(button);
    lv_obj_set_size(wings, 48, 7);
    lv_obj_align(wings, LV_ALIGN_CENTER, 0, -2);
    lv_obj_set_style_radius(wings, 4, 0);
    lv_obj_set_style_bg_color(wings, lv_color_white(), 0);
    lv_obj_set_style_border_width(wings, 0, 0);

    lv_obj_t *tail = lv_obj_create(button);
    lv_obj_set_size(tail, 24, 6);
    lv_obj_align(tail, LV_ALIGN_CENTER, 0, 16);
    lv_obj_set_style_radius(tail, 3, 0);
    lv_obj_set_style_bg_color(tail, lv_color_white(), 0);
    lv_obj_set_style_border_width(tail, 0, 0);
}

void add_rotation_lock_icon(lv_obj_t *button)
{
    lv_obj_t *shackle = lv_obj_create(button);
    lv_obj_set_size(shackle, 17, 17);
    lv_obj_align(shackle, LV_ALIGN_CENTER, 14, 3);
    lv_obj_set_style_radius(shackle, 8, 0);
    lv_obj_set_style_bg_opa(shackle, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(shackle, lv_color_white(), 0);
    lv_obj_set_style_border_width(shackle, 3, 0);

    lv_obj_t *body = lv_obj_create(button);
    lv_obj_set_size(body, 21, 16);
    lv_obj_align(body, LV_ALIGN_CENTER, 14, 12);
    lv_obj_set_style_radius(body, 3, 0);
    lv_obj_set_style_bg_color(body, lv_color_white(), 0);
    lv_obj_set_style_border_width(body, 0, 0);
}

void show_control_center_page(uint8_t page)
{
    control_center_page_index = page > 0 ? 1 : 0;
    const bool show_notifications = control_center_page_index == 1;
    lv_obj_set_flag(control_page, LV_OBJ_FLAG_HIDDEN, show_notifications);
    lv_obj_set_flag(notification_page, LV_OBJ_FLAG_HIDDEN, !show_notifications);
    lv_obj_set_size(control_page_dot, show_notifications ? 8 : 12, show_notifications ? 8 : 12);
    lv_obj_set_size(notification_page_dot, show_notifications ? 12 : 8, show_notifications ? 12 : 8);
    lv_obj_set_style_bg_opa(control_page_dot, show_notifications ? LV_OPA_40 : LV_OPA_COVER, 0);
    lv_obj_set_style_bg_opa(notification_page_dot, show_notifications ? LV_OPA_COVER : LV_OPA_40, 0);
    if (show_notifications) {
        notification_ui_dirty = true;
        refresh_notification_list();
    }
}

void create_notification_chrome()
{
    ESP_Brookesia_StatusBar *status_bar = phone != nullptr ? phone->getHome().getStatusBar() : nullptr;
    lv_obj_t *status_area = status_bar != nullptr ? status_bar->getAreaObj(1) : nullptr;
    if (status_area != nullptr) {
        notification_badge = lv_button_create(status_area);
    } else {
        notification_badge = lv_button_create(lv_layer_top());
    }
    lv_obj_set_size(notification_badge, 34, 28);
    lv_obj_set_style_radius(notification_badge, 0, 0);
    lv_obj_set_style_bg_opa(notification_badge, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(notification_badge, 0, 0);
    lv_obj_set_style_pad_all(notification_badge, 0, 0);
    lv_obj_set_style_shadow_width(notification_badge, 0, 0);
    lv_obj_add_event_cb(notification_badge, notification_badge_cb, LV_EVENT_CLICKED, nullptr);
    notification_badge_label = lv_label_create(notification_badge);
    lv_obj_set_style_text_font(notification_badge_label, &lv_font_montserrat_16, 0);
    lv_obj_center(notification_badge_label);
    lv_obj_add_flag(notification_badge, LV_OBJ_FLAG_HIDDEN);

    if (status_area != nullptr) {
        status_radio_icons = lv_label_create(status_area);
        lv_obj_set_style_text_font(status_radio_icons, &lv_font_montserrat_18, 0);
        status_charge_icon = lv_label_create(status_area);
        lv_label_set_text(status_charge_icon, LV_SYMBOL_USB);
        lv_obj_set_style_text_font(status_charge_icon, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(status_charge_icon, lv_color_white(), 0);
        lv_obj_add_flag(status_charge_icon, LV_OBJ_FLAG_HIDDEN);

        /* Keep every system indicator in the same flex row as Wi-Fi and the
         * battery. It can compress or clip at the left edge, but never overlap. */
        lv_obj_move_to_index(notification_badge, 0);
        lv_obj_move_to_index(status_bar->getWifiIconObj(), 1);
        lv_obj_move_to_index(status_radio_icons, 2);
        lv_obj_move_to_index(status_bar->getBatteryLabelObj(), 3);
        lv_obj_move_to_index(status_bar->getBatteryIconObj(), 4);
        lv_obj_move_to_index(status_charge_icon, 5);
    }

    fullscreen_notification_badge = lv_button_create(lv_layer_top());
    lv_obj_set_size(fullscreen_notification_badge, 34, 28);
    lv_obj_set_pos(fullscreen_notification_badge, 205, 4);
    lv_obj_set_style_radius(fullscreen_notification_badge, 0, 0);
    lv_obj_set_style_bg_opa(fullscreen_notification_badge, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(fullscreen_notification_badge, 0, 0);
    lv_obj_set_style_pad_all(fullscreen_notification_badge, 0, 0);
    lv_obj_set_style_shadow_width(fullscreen_notification_badge, 0, 0);
    lv_obj_add_event_cb(fullscreen_notification_badge, notification_badge_cb, LV_EVENT_CLICKED, nullptr);
    fullscreen_notification_badge_label = lv_label_create(fullscreen_notification_badge);
    lv_obj_set_style_text_font(fullscreen_notification_badge_label, &lv_font_montserrat_16, 0);
    lv_obj_center(fullscreen_notification_badge_label);
    lv_obj_add_flag(fullscreen_notification_badge, LV_OBJ_FLAG_HIDDEN);

    notification_banner = lv_button_create(lv_layer_top());
    lv_obj_set_size(notification_banner, 400, 92);
    lv_obj_align(notification_banner, LV_ALIGN_TOP_MID, 0, 48);
    lv_obj_set_style_radius(notification_banner, 8, 0);
    lv_obj_set_style_bg_color(notification_banner, lv_color_hex(0x263F52), 0);
    lv_obj_set_style_border_color(notification_banner, lv_color_hex(0x4E718A), 0);
    lv_obj_set_style_border_width(notification_banner, 1, 0);
    lv_obj_set_style_pad_all(notification_banner, 12, 0);
    lv_obj_set_flex_flow(notification_banner, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(notification_banner, 4, 0);
    lv_obj_add_event_cb(notification_banner, notification_banner_cb, LV_EVENT_CLICKED, nullptr);
    notification_banner_title = lv_label_create(notification_banner);
    lv_obj_set_width(notification_banner_title, LV_PCT(100));
    lv_obj_set_style_text_font(notification_banner_title, &lv_font_montserrat_18, 0);
    notification_banner_message = lv_label_create(notification_banner);
    lv_obj_set_width(notification_banner_message, LV_PCT(100));
    lv_label_set_long_mode(notification_banner_message, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(notification_banner_message, lv_color_hex(0xC6D5E0), 0);
    lv_obj_add_flag(notification_banner, LV_OBJ_FLAG_HIDDEN);
}

void create_control_center()
{
    control_center = lv_obj_create(lv_layer_top());
    lv_obj_set_size(control_center, 480, 480);
    lv_obj_set_pos(control_center, 0, 0);
    lv_obj_set_style_bg_color(control_center, lv_color_hex(0x10171D), 0);
    lv_obj_set_style_bg_opa(control_center, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(control_center, 0, 0);
    /* The framework status bar remains above this system overlay. Keep page
     * content below it so the first row is never hidden by the 40px bar. */
    lv_obj_set_style_pad_left(control_center, 18, 0);
    lv_obj_set_style_pad_right(control_center, 18, 0);
    lv_obj_set_style_pad_bottom(control_center, 18, 0);
    lv_obj_set_style_pad_top(control_center, 58, 0);
    lv_obj_set_flex_flow(control_center, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(control_center, 12, 0);
    lv_obj_add_flag(control_center, LV_OBJ_FLAG_HIDDEN);

    control_page = lv_obj_create(control_center);
    lv_obj_set_width(control_page, LV_PCT(100));
    lv_obj_set_flex_grow(control_page, 1);
    lv_obj_set_style_bg_opa(control_page, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(control_page, 0, 0);
    lv_obj_set_style_pad_all(control_page, 0, 0);
    lv_obj_set_flex_flow(control_page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(control_page, 12, 0);

    lv_obj_t *row = lv_obj_create(control_page);
    lv_obj_set_size(row, LV_PCT(100), 86);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_gap(row, 10, 0);
    wifi_button = add_control_tile(row, LV_SYMBOL_WIFI, wifi_button_cb, &wifi_button_label);
    ble_button = add_control_tile(row, LV_SYMBOL_BLUETOOTH, ble_button_cb, &ble_button_label);
    airplane_button = add_control_tile(row, "", airplane_button_cb, &airplane_button_label);
    add_airplane_icon(airplane_button);
    rotation_button = add_control_tile(row, LV_SYMBOL_REFRESH, rotation_button_cb, &rotation_button_label);
    add_rotation_lock_icon(rotation_button);

    lv_obj_t *brightness_title = lv_label_create(control_page);
    lv_label_set_text(brightness_title, "Luminosite");
    brightness_slider = lv_slider_create(control_page);
    lv_obj_set_width(brightness_slider, LV_PCT(100));
    lv_slider_set_range(brightness_slider, 1, 100);
    lv_obj_add_event_cb(brightness_slider, brightness_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    battery_label = lv_label_create(control_page);
    state_label = lv_label_create(control_page);
    time_label = lv_label_create(control_page);
    lv_obj_set_style_text_color(battery_label, lv_color_hex(0xC6D5E0), 0);
    lv_obj_set_style_text_color(state_label, lv_color_hex(0xC6D5E0), 0);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0xC6D5E0), 0);

    notification_page = lv_obj_create(control_center);
    lv_obj_set_width(notification_page, LV_PCT(100));
    lv_obj_set_flex_grow(notification_page, 1);
    lv_obj_set_style_bg_opa(notification_page, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(notification_page, 0, 0);
    lv_obj_set_style_pad_all(notification_page, 0, 0);
    lv_obj_set_flex_flow(notification_page, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(notification_page, 8, 0);
    lv_obj_add_flag(notification_page, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *notification_header = lv_obj_create(notification_page);
    lv_obj_set_size(notification_header, LV_PCT(100), 46);
    lv_obj_set_style_bg_opa(notification_header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(notification_header, 0, 0);
    lv_obj_set_style_pad_all(notification_header, 0, 0);
    lv_obj_set_flex_flow(notification_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(notification_header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *notification_title = lv_label_create(notification_header);
    lv_label_set_text(notification_title, "Toutes les notifications");
    lv_obj_t *clear_all = add_button(notification_header, LV_SYMBOL_TRASH, clear_all_notifications_cb, nullptr);
    lv_obj_set_size(clear_all, 48, 42);

    notification_list = lv_obj_create(notification_page);
    lv_obj_set_width(notification_list, LV_PCT(100));
    lv_obj_set_flex_grow(notification_list, 1);
    lv_obj_set_style_radius(notification_list, 6, 0);
    lv_obj_set_style_bg_color(notification_list, lv_color_hex(0x182128), 0);
    lv_obj_set_flex_flow(notification_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(notification_list, 6, 0);

    lv_obj_t *page_indicator = lv_obj_create(control_center);
    lv_obj_set_size(page_indicator, LV_PCT(100), 18);
    lv_obj_set_style_bg_opa(page_indicator, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(page_indicator, 0, 0);
    lv_obj_set_style_pad_all(page_indicator, 0, 0);
    lv_obj_set_style_pad_column(page_indicator, 10, 0);
    lv_obj_set_flex_flow(page_indicator, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(page_indicator, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    control_page_dot = lv_obj_create(page_indicator);
    notification_page_dot = lv_obj_create(page_indicator);
    for (lv_obj_t *dot : {control_page_dot, notification_page_dot}) {
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, lv_color_white(), 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    }
    show_control_center_page(0);
    create_notification_chrome();
    notification_ui_dirty = true;
    refresh_control_center();
}

void set_power_state(WaveshareSystemPowerState next)
{
    if (next == power_state) {
        return;
    }
    power_state = next;
    if (power_state == WAVESHARE_SYSTEM_ACTIVE) {
        lvgl_port_set_display_power(true);
    } else if (power_state == WAVESHARE_SYSTEM_SCREEN_OFF || power_state == WAVESHARE_SYSTEM_CONNECTED_STANDBY ||
               power_state == WAVESHARE_SYSTEM_DEEP_SLEEP_PENDING) {
        lvgl_port_set_display_power(false);
    }
    Serial.printf("[system] power state: %s\n", power_state_name(power_state));
    refresh_control_center();
}

void update_control_center_touch_gesture()
{
    if (!control_center_visible) {
        control_center_touch_active = false;
        return;
    }
    lv_indev_t *touch = lvgl_port_get_touch();
    if (touch == nullptr) return;

    lv_point_t point = {};
    lv_indev_get_point(touch, &point);
    if (lv_indev_get_state(touch) == LV_INDEV_STATE_PRESSED) {
        if (!control_center_touch_active) {
            control_center_touch_active = true;
            control_center_touch_start = point;
        }
        return;
    }
    if (!control_center_touch_active) return;
    control_center_touch_active = false;

    const int dx = point.x - control_center_touch_start.x;
    const int dy = point.y - control_center_touch_start.y;
    if (abs(dx) < 54 || abs(dx) <= abs(dy)) return;

    lv_area_t slider_area = {};
    lv_obj_get_coords(brightness_slider, &slider_area);
    const bool started_on_brightness = control_center_page_index == 0 &&
        control_center_touch_start.x >= slider_area.x1 && control_center_touch_start.x <= slider_area.x2 &&
        control_center_touch_start.y >= slider_area.y1 && control_center_touch_start.y <= slider_area.y2;
    if (started_on_brightness) return;

    show_control_center_page(dx < 0 ? 1 : 0);
}

void gesture_cb(lv_event_t *event)
{
    if (phone == nullptr) {
        return;
    }
    const lv_event_code_t code = lv_event_get_code(event);
    ESP_Brookesia_Gesture *gesture = phone->getManager().getGesture();
    if (gesture == nullptr) {
        return;
    }
    if (code == gesture->getPressEventCode()) {
        waveshare_system_note_user_activity();
        return;
    }
    if (code != gesture->getReleaseEventCode()) {
        return;
    }
    auto *info = static_cast<ESP_Brookesia_GestureInfo_t *>(lv_event_get_param(event));
    if (info != nullptr && control_center_visible) {
        if (info->direction == ESP_BROOKESIA_GESTURE_DIR_UP) {
            waveshare_system_hide_control_center();
            return;
        }
        lv_area_t slider_area = {};
        lv_obj_get_coords(brightness_slider, &slider_area);
        const bool started_on_brightness = control_center_page_index == 0 &&
            info->start_x >= slider_area.x1 && info->start_x <= slider_area.x2 &&
            info->start_y >= slider_area.y1 && info->start_y <= slider_area.y2;
        if (!started_on_brightness && info->direction == ESP_BROOKESIA_GESTURE_DIR_LEFT) {
            show_control_center_page(1);
            return;
        }
        if (!started_on_brightness && info->direction == ESP_BROOKESIA_GESTURE_DIR_RIGHT) {
            show_control_center_page(0);
            return;
        }
    }
    if (info != nullptr && (info->start_area & ESP_BROOKESIA_GESTURE_AREA_TOP_EDGE) &&
            (info->direction == ESP_BROOKESIA_GESTURE_DIR_DOWN)) {
        waveshare_system_show_control_center();
    }
}

void handle_buttons()
{
    const uint32_t now = millis();
    const bool plus_down = digitalRead(PIN_KEY_PLUS) == LOW;
    const bool minus_down = digitalRead(PIN_KEY_MINUS) == LOW;

    if (plus_down && !key_plus_was_down) key_plus_down_ms = now;
    if (!plus_down && key_plus_was_down) {
        waveshare_system_note_user_activity();
        if (phone != nullptr) {
            ESP_Brookesia_AppLauncher *launcher = phone->getHome().getAppLauncher();
            if (launcher != nullptr) {
                if (now - key_plus_down_ms < LONG_PRESS_MS) {
                    launcher->selectNextIcon();
                } else {
                    launcher->startSelectedIcon();
                }
            }
        }
    }
    key_plus_was_down = plus_down;

    if (minus_down && !key_minus_was_down) key_minus_down_ms = now;
    if (!minus_down && key_minus_was_down) {
        waveshare_system_note_user_activity();
        if (phone != nullptr) {
            if (now - key_minus_down_ms < LONG_PRESS_MS) {
                ESP_Brookesia_AppLauncher *launcher = phone->getHome().getAppLauncher();
                if (launcher != nullptr) launcher->selectPreviousIcon();
            } else {
                phone->sendNavigateEvent(ESP_BROOKESIA_CORE_NAVIGATE_TYPE_HOME);
            }
        }
    }
    key_minus_was_down = minus_down;
}

} // namespace

bool waveshare_system_post_notification(int app_id, const char *title, const char *message)
{
    if (title == nullptr || message == nullptr) return false;
    portENTER_CRITICAL(&service_mux);
    int slot = -1;
    uint32_t oldest = UINT32_MAX;
    for (uint8_t i = 0; i < MAX_NOTIFICATIONS; i++) {
        if (!notifications[i].used) {
            slot = i;
            break;
        }
        if (notifications[i].created_ms < oldest) {
            oldest = notifications[i].created_ms;
            slot = i;
        }
    }
    NotificationEntry &entry = notifications[slot];
    entry.used = true;
    entry.unread = true;
    entry.app_id = app_id;
    entry.created_ms = millis();
    strlcpy(entry.title, title, sizeof(entry.title));
    strlcpy(entry.message, message, sizeof(entry.message));
    notification_ui_dirty = true;
    notification_wake_pending = true;
    pending_banner_notification = entry;
    notification_banner_pending = true;
    portEXIT_CRITICAL(&service_mux);
    return true;
}

void waveshare_system_clear_notifications(int app_id)
{
    portENTER_CRITICAL(&service_mux);
    for (uint8_t i = 0; i < MAX_NOTIFICATIONS; i++) {
        if (notifications[i].used && notifications[i].app_id == app_id) notifications[i].used = false;
    }
    notification_ui_dirty = true;
    portEXIT_CRITICAL(&service_mux);
}

uint8_t waveshare_system_get_notification_count(void)
{
    uint8_t count = 0;
    portENTER_CRITICAL(&service_mux);
    for (uint8_t i = 0; i < MAX_NOTIFICATIONS; i++) if (notifications[i].used) count++;
    portEXIT_CRITICAL(&service_mux);
    return count;
}

bool waveshare_system_schedule_job(int app_id, uint32_t period_ms, WaveshareScheduledJobCallback callback,
                                   void *context, bool run_immediately)
{
    if (callback == nullptr || period_ms < 1000) return false;
    for (uint8_t i = 0; i < MAX_SCHEDULED_JOBS; i++) {
        ScheduledJob &job = scheduled_jobs[i];
        if (job.used && job.app_id == app_id && job.callback == callback) {
            job.period_ms = period_ms;
            job.next_run_ms = millis() + (run_immediately ? 0 : period_ms);
            job.context = context;
            return true;
        }
        if (!job.used) {
            job.used = true;
            job.running = false;
            job.app_id = app_id;
            job.period_ms = period_ms;
            job.next_run_ms = millis() + (run_immediately ? 0 : period_ms);
            job.callback = callback;
            job.context = context;
            return true;
        }
    }
    return false;
}

void waveshare_system_cancel_jobs(int app_id)
{
    for (uint8_t i = 0; i < MAX_SCHEDULED_JOBS; i++) {
        if (scheduled_jobs[i].used && scheduled_jobs[i].app_id == app_id && !scheduled_jobs[i].running) {
            scheduled_jobs[i].used = false;
        }
    }
}

uint32_t waveshare_system_get_next_job_delay_ms(void)
{
    const uint32_t now = millis();
    uint32_t result = UINT32_MAX;
    for (uint8_t i = 0; i < MAX_SCHEDULED_JOBS; i++) {
        if (!scheduled_jobs[i].used) continue;
        const int32_t remaining = static_cast<int32_t>(scheduled_jobs[i].next_run_ms - now);
        if (remaining <= 0) return 0;
        if (static_cast<uint32_t>(remaining) < result) result = remaining;
    }
    return result;
}

void waveshare_system_request_ntp_sync(void)
{
    ntp_started = false;
    time_synchronized = false;
    last_ntp_attempt_ms = 0;
}

bool waveshare_system_time_is_synchronized(void)
{
    return time_synchronized;
}

bool waveshare_system_ble_is_allowed(void)
{
    return settings.ble_enabled && !settings.airplane_mode;
}

bool waveshare_system_acquire_display_lease(const char *)
{
    if (display_lease_count < UINT8_MAX) display_lease_count++;
    waveshare_system_note_user_activity();
    return true;
}

void waveshare_system_release_display_lease(const char *)
{
    if (display_lease_count > 0) display_lease_count--;
}

bool waveshare_system_begin(ESP_Brookesia_Phone *phone_in)
{
    phone = phone_in;
    if (phone == nullptr) return false;
    load_settings();
    pinMode(PIN_KEY_PLUS, INPUT_PULLUP);
    pinMode(PIN_KEY_MINUS, INPUT_PULLUP);
    lvgl_port_set_brightness(settings.brightness);
    apply_radio_settings();
#if WAVESHARE_SYSTEM_HAS_PMU
    init_pmu();
#endif
#if WAVESHARE_SYSTEM_HAS_IMU
    init_imu();
#endif
    create_control_center();
    ESP_Brookesia_Gesture *gesture = phone->getManager().getGesture();
    if (gesture == nullptr) return false;
    lv_obj_add_event_cb(gesture->getEventObj(), gesture_cb, gesture->getPressEventCode(), nullptr);
    lv_obj_add_event_cb(gesture->getEventObj(), gesture_cb, gesture->getReleaseEventCode(), nullptr);
    last_user_activity_ms = millis();
    Serial.println("[system] Waveshare services ready");
    return true;
}

void waveshare_system_note_user_activity(void)
{
    notification_wake_active = false;
    last_user_activity_ms = millis();
    if (power_state != WAVESHARE_SYSTEM_ACTIVE) set_power_state(WAVESHARE_SYSTEM_ACTIVE);
}

void waveshare_system_show_control_center(void)
{
    if (control_center == nullptr) return;
    waveshare_system_note_user_activity();
    control_center_visible = true;
    if (phone != nullptr) phone->getManager().setSystemOverlayVisible(true);
    lv_obj_move_foreground(control_center);
    lv_obj_clear_flag(control_center, LV_OBJ_FLAG_HIDDEN);
    refresh_control_center();
}

void waveshare_system_hide_control_center(void)
{
    if (control_center == nullptr) return;
    control_center_visible = false;
    if (phone != nullptr) phone->getManager().setSystemOverlayVisible(false);
    lv_obj_add_flag(control_center, LV_OBJ_FLAG_HIDDEN);
}

WaveshareSystemPowerState waveshare_system_get_power_state(void)
{
    return power_state;
}

void waveshare_system_tick(void)
{
    const uint32_t now = millis();
    if (now - last_tick_ms < SYSTEM_TICK_PERIOD_MS) return;
    last_tick_ms = now;
    dispatch_scheduled_jobs(now);
    update_network_time(now);
    if (notification_wake_pending) {
        notification_wake_pending = false;
        if (power_state != WAVESHARE_SYSTEM_ACTIVE) {
            set_power_state(WAVESHARE_SYSTEM_ACTIVE);
            notification_wake_active = true;
            notification_wake_deadline_ms = now + NOTIFICATION_WAKE_DELAY_MS;
        }
    }
    if (notification_banner_pending) {
        NotificationEntry banner = {};
        portENTER_CRITICAL(&service_mux);
        banner = pending_banner_notification;
        notification_banner_pending = false;
        portEXIT_CRITICAL(&service_mux);
        notification_banner_app_id = banner.app_id;
        notification_banner_deadline_ms = now + NOTIFICATION_BANNER_DELAY_MS;
        lv_label_set_text_fmt(notification_banner_title, LV_SYMBOL_BELL "  %s", banner.title);
        lv_label_set_text(notification_banner_message, banner.message);
        lv_obj_move_foreground(notification_banner);
        lv_obj_clear_flag(notification_banner, LV_OBJ_FLAG_HIDDEN);
    }
    refresh_status_indicators();
    update_control_center_touch_gesture();
    handle_buttons();
    if (control_center_visible || notification_ui_dirty) refresh_control_center();
#if WAVESHARE_SYSTEM_HAS_PMU
    poll_pmu_power_key(now);
    update_pmu_thermal_policy(now);
#endif
#if WAVESHARE_SYSTEM_HAS_IMU
    poll_auto_rotation(now);
#endif
    if (notification_wake_active && static_cast<int32_t>(now - notification_wake_deadline_ms) >= 0) {
        notification_wake_active = false;
        set_power_state(WAVESHARE_SYSTEM_DEEP_SLEEP_PENDING);
        return;
    }
    if (notification_banner_deadline_ms != 0 && static_cast<int32_t>(now - notification_banner_deadline_ms) >= 0) {
        notification_banner_deadline_ms = 0;
        lv_obj_add_flag(notification_banner, LV_OBJ_FLAG_HIDDEN);
    }
    /* Preserve the 30-second notification wake window instead of immediately
     * reapplying the older idle deadline that preceded the notification. */
    if (notification_wake_active) return;
    if (display_lease_count > 0 || control_center_visible) return;

    /* Re-read millis after input processing. A PWR event can update
     * last_user_activity_ms later than the tick's original `now`; subtracting
     * from that stale value underflows and immediately turns the display off. */
    const uint32_t idle_ms = millis() - last_user_activity_ms;
    if (idle_ms >= DEEP_SLEEP_DELAY_MS) {
        set_power_state(WAVESHARE_SYSTEM_DEEP_SLEEP_PENDING);
        /* Deliberately no esp_deep_sleep_start here. The PMU power path and wake
         * sources must be validated on hardware before enabling irreversible sleep. */
    } else if (idle_ms >= CONNECTED_STANDBY_DELAY_MS) {
        set_power_state(WAVESHARE_SYSTEM_CONNECTED_STANDBY);
    } else if (idle_ms >= settings.display_off_delay_ms) {
        set_power_state(WAVESHARE_SYSTEM_SCREEN_OFF);
    }
}

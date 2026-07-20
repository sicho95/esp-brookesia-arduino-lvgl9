/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <stdint.h>

class ESP_Brookesia_Phone;

typedef void (*WaveshareScheduledJobCallback)(void *context);

enum WaveshareSystemPowerState : uint8_t {
    WAVESHARE_SYSTEM_ACTIVE,
    WAVESHARE_SYSTEM_SCREEN_OFF,
    WAVESHARE_SYSTEM_CONNECTED_STANDBY,
    WAVESHARE_SYSTEM_DEEP_SLEEP_PENDING,
};

/* A foreground app owns a display lease while its information must stay visible. */
bool waveshare_system_acquire_display_lease(const char *owner);
void waveshare_system_release_display_lease(const char *owner);

/* Notifications are copied into a fixed-size service queue. app_id < 0 creates
 * a system notification; otherwise tapping it starts/resumes that app. */
bool waveshare_system_post_notification(int app_id, const char *title, const char *message);
void waveshare_system_clear_notifications(int app_id);
uint8_t waveshare_system_get_notification_count(void);

/* Periodic callbacks run on a background FreeRTOS task and must not call LVGL.
 * They may post a notification through the thread-safe API above. */
bool waveshare_system_schedule_job(int app_id, uint32_t period_ms, WaveshareScheduledJobCallback callback,
                                   void *context, bool run_immediately = false);
void waveshare_system_cancel_jobs(int app_id);
uint32_t waveshare_system_get_next_job_delay_ms(void);

void waveshare_system_request_ntp_sync(void);
bool waveshare_system_time_is_synchronized(void);
bool waveshare_system_ble_is_allowed(void);

/* Hardware audio service: ES7210 dual microphones and ES8311 speaker codec. */
bool waveshare_system_microphones_enabled(void);
uint8_t waveshare_system_microphone_gain(void);
uint8_t waveshare_system_output_volume(void);

bool waveshare_system_begin(ESP_Brookesia_Phone *phone);
void waveshare_system_tick(void);
void waveshare_system_note_user_activity(void);
void waveshare_system_show_control_center(void);
void waveshare_system_hide_control_center(void);
WaveshareSystemPowerState waveshare_system_get_power_state(void);

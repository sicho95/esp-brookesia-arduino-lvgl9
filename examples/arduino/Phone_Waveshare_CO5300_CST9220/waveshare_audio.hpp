/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include <Arduino.h>

bool waveshare_audio_begin(void);
void waveshare_audio_suspend(void);
bool waveshare_audio_resume(void);

bool waveshare_audio_set_microphones_enabled(bool enabled);
bool waveshare_audio_microphones_enabled(void);
bool waveshare_audio_set_microphone_gain(uint8_t gain_step);
uint8_t waveshare_audio_get_microphone_gain(void);

bool waveshare_audio_set_output_volume(uint8_t volume);
uint8_t waveshare_audio_get_output_volume(void);

size_t waveshare_audio_read(void *buffer, size_t bytes);
size_t waveshare_audio_write(const void *buffer, size_t bytes);
bool waveshare_audio_is_ready(void);
bool waveshare_audio_noise_reduction_is_ready(void);
bool waveshare_audio_noise_reduction_is_available(void);

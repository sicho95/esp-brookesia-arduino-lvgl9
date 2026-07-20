/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 * SPDX-License-Identifier: CC0-1.0
 */
#include "waveshare_audio.hpp"

#include <ESP_I2S.h>
#include <Wire.h>

namespace {

constexpr uint8_t ES8311_ADDRESS = 0x18;
constexpr uint8_t ES7210_ADDRESS = 0x40;
constexpr int PIN_I2S_MCLK = 42;
constexpr int PIN_I2S_BCLK = 9;
constexpr int PIN_I2S_DIN = 10;
constexpr int PIN_I2S_WS = 45;
constexpr int PIN_I2S_DOUT = 8;
constexpr int PIN_SPEAKER_ENABLE = 46;
constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;

I2SClass audio_i2s;
bool audio_ready = false;
bool microphones_enabled = true;
uint8_t microphone_gain = 8; // 24 dB, ES7210 hardware step.
uint8_t output_volume = 60;

bool write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

uint8_t read_register(uint8_t address, uint8_t reg)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return 0;
    return Wire.read();
}

bool probe(uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool init_es8311()
{
    if (!probe(ES8311_ADDRESS)) return false;
    // 16 kHz, 16-bit stereo slave, MCLK = 256 * Fs. This is the board vendor's clock plan.
    const uint8_t init[][2] = {
        {0x00, 0x1F}, {0x00, 0x00}, {0x00, 0x80},
        {0x01, 0x3F}, {0x02, 0x00}, {0x03, 0x10}, {0x04, 0x10},
        {0x05, 0x00}, {0x06, 0x03}, {0x07, 0x00}, {0x08, 0xFF},
        {0x09, 0x0C}, {0x0A, 0x0C}, {0x0D, 0x01}, {0x0E, 0x02},
        {0x12, 0x00}, {0x13, 0x10}, {0x1C, 0x6A}, {0x37, 0x08},
        {0x31, 0x00},
    };
    for (const auto &entry : init) if (!write_register(ES8311_ADDRESS, entry[0], entry[1])) return false;
    return waveshare_audio_set_output_volume(output_volume);
}

bool set_es7210_power(bool enabled)
{
    if (!probe(ES7210_ADDRESS)) return false;
    if (!enabled) {
        for (uint8_t reg = 0x47; reg <= 0x4A; reg++) write_register(ES7210_ADDRESS, reg, 0xFF);
        write_register(ES7210_ADDRESS, 0x4B, 0xFF);
        write_register(ES7210_ADDRESS, 0x4C, 0xFF);
        write_register(ES7210_ADDRESS, 0x01, 0x7F);
        return write_register(ES7210_ADDRESS, 0x06, 0x07);
    }
    write_register(ES7210_ADDRESS, 0x01, 0x1F);
    write_register(ES7210_ADDRESS, 0x06, 0x00);
    // The two fitted microphones are connected to ES7210 channels 3 and 4.
    write_register(ES7210_ADDRESS, 0x49, 0x00);
    write_register(ES7210_ADDRESS, 0x4A, 0x00);
    write_register(ES7210_ADDRESS, 0x4B, 0x0F);
    write_register(ES7210_ADDRESS, 0x4C, 0x00);
    return waveshare_audio_set_microphone_gain(microphone_gain);
}

bool init_es7210()
{
    if (!probe(ES7210_ADDRESS)) return false;
    const uint8_t init[][2] = {
        {0x00, 0xFF}, {0x00, 0x41}, {0x01, 0x1F},
        {0x09, 0x30}, {0x0A, 0x30}, {0x40, 0xC3},
        {0x41, 0x70}, {0x42, 0x70}, {0x07, 0x20},
        {0x02, 0xC1}, {0x04, 0x01}, {0x05, 0x00},
        {0x44, 0x10}, {0x45, 0x10}, {0x46, 0x10},
    };
    for (const auto &entry : init) if (!write_register(ES7210_ADDRESS, entry[0], entry[1])) return false;
    return set_es7210_power(microphones_enabled);
}

} // namespace

bool waveshare_audio_begin(void)
{
    pinMode(PIN_SPEAKER_ENABLE, OUTPUT);
    digitalWrite(PIN_SPEAKER_ENABLE, LOW);
    audio_i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN, PIN_I2S_MCLK);
    if (!audio_i2s.begin(I2S_MODE_STD, AUDIO_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                         I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
        Serial.println("[audio] I2S initialization failed");
        return false;
    }
    const bool speaker_ok = init_es8311();
    const bool microphones_ok = init_es7210();
    digitalWrite(PIN_SPEAKER_ENABLE, speaker_ok ? HIGH : LOW);
    audio_ready = speaker_ok || microphones_ok;
    Serial.printf("[audio] ES8311=%s ES7210=%s\n", speaker_ok ? "OK" : "absent", microphones_ok ? "OK" : "absent");
    return audio_ready;
}

void waveshare_audio_suspend(void)
{
    if (!audio_ready) return;
    write_register(ES8311_ADDRESS, 0x31, read_register(ES8311_ADDRESS, 0x31) | 0x60);
    digitalWrite(PIN_SPEAKER_ENABLE, LOW);
    set_es7210_power(false);
}

bool waveshare_audio_resume(void)
{
    if (!audio_ready) return waveshare_audio_begin();
    const bool speaker_ok = init_es8311();
    const bool microphones_ok = set_es7210_power(microphones_enabled);
    digitalWrite(PIN_SPEAKER_ENABLE, speaker_ok ? HIGH : LOW);
    return speaker_ok || microphones_ok;
}

bool waveshare_audio_set_microphones_enabled(bool enabled)
{
    microphones_enabled = enabled;
    return set_es7210_power(enabled);
}

bool waveshare_audio_microphones_enabled(void) { return microphones_enabled; }

bool waveshare_audio_set_microphone_gain(uint8_t gain_step)
{
    microphone_gain = constrain(gain_step, 0, 14);
    if (!microphones_enabled || !probe(ES7210_ADDRESS)) return true;
    const uint8_t gain = 0x10 | microphone_gain;
    return write_register(ES7210_ADDRESS, 0x45, gain) && write_register(ES7210_ADDRESS, 0x46, gain);
}

uint8_t waveshare_audio_get_microphone_gain(void) { return microphone_gain; }

bool waveshare_audio_set_output_volume(uint8_t volume)
{
    output_volume = constrain(volume, 0, 100);
    if (!probe(ES8311_ADDRESS)) return false;
    const uint8_t codec_volume = output_volume == 0 ? 0 : static_cast<uint8_t>((output_volume * 256U) / 100U - 1U);
    return write_register(ES8311_ADDRESS, 0x32, codec_volume);
}

uint8_t waveshare_audio_get_output_volume(void) { return output_volume; }

size_t waveshare_audio_read(void *buffer, size_t bytes)
{
    if (!audio_ready || !microphones_enabled || buffer == nullptr) return 0;
    return audio_i2s.readBytes(static_cast<char *>(buffer), bytes);
}

size_t waveshare_audio_write(const void *buffer, size_t bytes)
{
    if (!audio_ready || buffer == nullptr || output_volume == 0) return 0;
    return audio_i2s.write(static_cast<const uint8_t *>(buffer), bytes);
}

bool waveshare_audio_is_ready(void) { return audio_ready; }

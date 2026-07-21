/*
 * SPDX-FileCopyrightText: 2026 Sicho95
 * SPDX-License-Identifier: CC0-1.0
 */
#include "waveshare_audio.hpp"

#include <ESP_I2S.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#if __has_include(<esp_afe_config.h>) && __has_include(<esp_afe_sr_models.h>)
extern "C" {
#include <esp_afe_config.h>
#include <esp_afe_sr_models.h>
}
#define WAVESHARE_AUDIO_HAS_AFE 1
#else
#define WAVESHARE_AUDIO_HAS_AFE 0
#endif

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
constexpr size_t PLAYBACK_REFERENCE_SAMPLES = 2048;
constexpr size_t MIN_INTERNAL_HEAP_AFTER_AFE = 32 * 1024;

I2SClass audio_i2s;
bool audio_ready = false;
bool microphones_enabled = true;
uint8_t microphone_gain = 8; // 24 dB, ES7210 hardware step.
uint8_t output_volume = 60;
portMUX_TYPE reference_mux = portMUX_INITIALIZER_UNLOCKED;
int16_t *playback_reference = nullptr;
size_t reference_read_index = 0;
size_t reference_write_index = 0;
size_t reference_count = 0;
SemaphoreHandle_t audio_read_mutex = nullptr;

#if WAVESHARE_AUDIO_HAS_AFE
afe_config_t *afe_config = nullptr;
const esp_afe_sr_iface_t *afe_interface = nullptr;
esp_afe_sr_data_t *afe_data = nullptr;
int afe_feed_samples = 0;
int afe_fetch_samples = 0;
int16_t *afe_raw_stereo = nullptr;
int16_t *afe_feed_buffer = nullptr;
int16_t *afe_output_cache = nullptr;
size_t afe_output_bytes = 0;
size_t afe_output_offset = 0;
#endif

void *allocate_audio_buffer(size_t bytes)
{
    void *buffer = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return buffer != nullptr ? buffer : malloc(bytes);
}

void clear_playback_reference()
{
    portENTER_CRITICAL(&reference_mux);
    reference_read_index = 0;
    reference_write_index = 0;
    reference_count = 0;
    portEXIT_CRITICAL(&reference_mux);
}

void push_playback_reference(const int16_t *stereo, size_t frames)
{
    if (stereo == nullptr || playback_reference == nullptr) return;
    portENTER_CRITICAL(&reference_mux);
    for (size_t i = 0; i < frames; i++) {
        const int32_t mono = (static_cast<int32_t>(stereo[i * 2]) + stereo[i * 2 + 1]) / 2;
        playback_reference[reference_write_index] = static_cast<int16_t>(mono);
        reference_write_index = (reference_write_index + 1) % PLAYBACK_REFERENCE_SAMPLES;
        if (reference_count < PLAYBACK_REFERENCE_SAMPLES) reference_count++;
        else reference_read_index = (reference_read_index + 1) % PLAYBACK_REFERENCE_SAMPLES;
    }
    portEXIT_CRITICAL(&reference_mux);
}

int16_t pop_playback_reference()
{
    if (playback_reference == nullptr) return 0;
    portENTER_CRITICAL(&reference_mux);
    int16_t sample = 0;
    if (reference_count > 0) {
        sample = playback_reference[reference_read_index];
        reference_read_index = (reference_read_index + 1) % PLAYBACK_REFERENCE_SAMPLES;
        reference_count--;
    }
    portEXIT_CRITICAL(&reference_mux);
    return sample;
}

#if WAVESHARE_AUDIO_HAS_AFE
void destroy_audio_frontend()
{
    if (afe_interface != nullptr && afe_data != nullptr) afe_interface->destroy(afe_data);
    afe_data = nullptr;
    afe_interface = nullptr;
    if (afe_config != nullptr) afe_config_free(afe_config);
    afe_config = nullptr;
    free(afe_raw_stereo);
    free(afe_feed_buffer);
    free(afe_output_cache);
    free(playback_reference);
    afe_raw_stereo = nullptr;
    afe_feed_buffer = nullptr;
    afe_output_cache = nullptr;
    playback_reference = nullptr;
    afe_output_bytes = 0;
    afe_output_offset = 0;
}

bool init_audio_frontend()
{
    if (afe_data != nullptr) return true;
    // MMR = two ES7210 microphones plus the exact playback stream as AEC reference.
    afe_config = afe_config_init("MMR", nullptr, AFE_TYPE_VC, AFE_MODE_LOW_COST);
    if (afe_config == nullptr) return false;
    afe_config->aec_init = true;
    afe_config->aec_mode = AEC_MODE_VOIP_LOW_COST;
    afe_config->se_init = true;
    afe_config->ns_init = true;
    afe_config->vad_init = true;
    afe_config->vad_mode = VAD_MODE_3;
    afe_config->wakenet_init = false;
    afe_config->agc_init = true;
    afe_config->agc_compression_gain_db = 6;
    afe_config->agc_target_level_dbfs = 6;
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config->afe_perferred_core = 0;
    afe_config->afe_perferred_priority = 5;
    afe_config = afe_config_check(afe_config);
    afe_interface = esp_afe_handle_from_config(afe_config);
    if (afe_interface == nullptr) {
        destroy_audio_frontend();
        return false;
    }
    afe_data = afe_interface->create_from_config(afe_config);
    if (afe_data == nullptr) {
        destroy_audio_frontend();
        return false;
    }
    afe_feed_samples = afe_interface->get_feed_chunksize(afe_data);
    afe_fetch_samples = afe_interface->get_fetch_chunksize(afe_data);
    if (afe_feed_samples <= 0 || afe_fetch_samples <= 0 || afe_interface->get_feed_channel_num(afe_data) != 3) {
        destroy_audio_frontend();
        return false;
    }
    afe_raw_stereo = static_cast<int16_t *>(allocate_audio_buffer(afe_feed_samples * 2 * sizeof(int16_t)));
    afe_feed_buffer = static_cast<int16_t *>(allocate_audio_buffer(afe_feed_samples * 3 * sizeof(int16_t)));
    afe_output_cache = static_cast<int16_t *>(allocate_audio_buffer(afe_fetch_samples * sizeof(int16_t)));
    playback_reference = static_cast<int16_t *>(allocate_audio_buffer(PLAYBACK_REFERENCE_SAMPLES * sizeof(int16_t)));
    if (afe_raw_stereo == nullptr || afe_feed_buffer == nullptr || afe_output_cache == nullptr ||
            playback_reference == nullptr) {
        destroy_audio_frontend();
        return false;
    }
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < MIN_INTERNAL_HEAP_AFTER_AFE) {
        Serial.println("[audio] AFE disabled: internal SRAM reserve would be exhausted");
        destroy_audio_frontend();
        return false;
    }
    afe_interface->print_pipeline(afe_data);
    Serial.printf("[audio] AFE ready: AEC+dual-mic enhancement+VAD+AGC, feed=%d fetch=%d\n",
                  afe_feed_samples, afe_fetch_samples);
    return true;
}

bool process_audio_frontend_frame()
{
    if (afe_data == nullptr || !microphones_enabled) return false;
    const size_t raw_bytes = afe_feed_samples * 2 * sizeof(int16_t);
    if (audio_i2s.readBytes(reinterpret_cast<char *>(afe_raw_stereo), raw_bytes) != raw_bytes) return false;
    for (int i = 0; i < afe_feed_samples; i++) {
        afe_feed_buffer[i * 3] = afe_raw_stereo[i * 2];
        afe_feed_buffer[i * 3 + 1] = afe_raw_stereo[i * 2 + 1];
        afe_feed_buffer[i * 3 + 2] = pop_playback_reference();
    }
    if (afe_interface->feed(afe_data, afe_feed_buffer) <= 0) return false;
    afe_fetch_result_t *result = afe_interface->fetch_with_delay(afe_data, pdMS_TO_TICKS(500));
    if (result == nullptr || result->ret_value != 0 || result->data == nullptr || result->data_size <= 0) return false;
    afe_output_bytes = min<size_t>(result->data_size, afe_fetch_samples * sizeof(int16_t));
    memcpy(afe_output_cache, result->data, afe_output_bytes);
    afe_output_offset = 0;
    return true;
}
#endif

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
    if (audio_read_mutex == nullptr) audio_read_mutex = xSemaphoreCreateMutex();
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
#if WAVESHARE_AUDIO_HAS_AFE
    const bool afe_available = microphones_ok;
#else
    const bool afe_available = false;
#endif
    Serial.printf("[audio] ES8311=%s ES7210=%s AFE=%s\n", speaker_ok ? "OK" : "absent",
                  microphones_ok ? "OK" : "absent", afe_available ? "deferred" : "absent");
    return audio_ready;
}

void waveshare_audio_suspend(void)
{
    if (!audio_ready) return;
    write_register(ES8311_ADDRESS, 0x31, read_register(ES8311_ADDRESS, 0x31) | 0x60);
    digitalWrite(PIN_SPEAKER_ENABLE, LOW);
    set_es7210_power(false);
#if WAVESHARE_AUDIO_HAS_AFE
    destroy_audio_frontend();
#endif
    clear_playback_reference();
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
#if WAVESHARE_AUDIO_HAS_AFE
    if (!enabled) destroy_audio_frontend();
#endif
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
    if (audio_read_mutex != nullptr && xSemaphoreTake(audio_read_mutex, portMAX_DELAY) != pdTRUE) return 0;
    size_t result_bytes = 0;
#if WAVESHARE_AUDIO_HAS_AFE
    if (afe_data == nullptr) init_audio_frontend();
    if (afe_data != nullptr) {
        size_t copied = 0;
        uint8_t *destination = static_cast<uint8_t *>(buffer);
        while (copied < bytes) {
            if (afe_output_offset >= afe_output_bytes && !process_audio_frontend_frame()) break;
            const size_t available = afe_output_bytes - afe_output_offset;
            const size_t amount = min(available, bytes - copied);
            memcpy(destination + copied, reinterpret_cast<uint8_t *>(afe_output_cache) + afe_output_offset, amount);
            copied += amount;
            afe_output_offset += amount;
        }
        result_bytes = copied;
    } else {
        result_bytes = audio_i2s.readBytes(static_cast<char *>(buffer), bytes);
    }
#else
    result_bytes = audio_i2s.readBytes(static_cast<char *>(buffer), bytes);
#endif
    if (audio_read_mutex != nullptr) xSemaphoreGive(audio_read_mutex);
    return result_bytes;
}

size_t waveshare_audio_write(const void *buffer, size_t bytes)
{
    if (!audio_ready || buffer == nullptr || output_volume == 0) return 0;
    const size_t written = audio_i2s.write(static_cast<const uint8_t *>(buffer), bytes);
    push_playback_reference(static_cast<const int16_t *>(buffer), written / (2 * sizeof(int16_t)));
    return written;
}

bool waveshare_audio_is_ready(void) { return audio_ready; }

bool waveshare_audio_noise_reduction_is_ready(void)
{
#if WAVESHARE_AUDIO_HAS_AFE
    return afe_data != nullptr;
#else
    return false;
#endif
}

bool waveshare_audio_noise_reduction_is_available(void)
{
#if WAVESHARE_AUDIO_HAS_AFE
    return audio_ready && microphones_enabled;
#else
    return false;
#endif
}

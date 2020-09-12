/**
 * Text To Speech synthesis library
 * Copyright (c) 2008 Clive Webster. All rights reserved.
 *
 * Nov. 29th 2009 - Modified to work with Arduino by Gabriel Petrut:
 * The Text To Speech library uses Timer1 to generate the PWM
 * output on digital pin 10. The output signal needs to be fed
 * to an RC filter then through an amplifier to the speaker.
 * http://www.tehnorama.ro/minieric-modulul-de-control-si-sinteza-vocala
 *
 * Modified to allow use of different PWM pins by Stephen Crane.
 * Modified for use with Teensy Audio Library by Timo Sandmann.
 */

#pragma once

#include "english.h"

#include "arduino_freertos.h"
#include "AudioStream.h"
#include "circular_buffer.h"
#include <string>
#include <string_view>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>


class TTS : public AudioStream {
public:
    FLASHMEM TTS();

    FLASHMEM virtual ~TTS();

    bool speak(const std::string_view& text, const bool block);

    uint8_t get_pitch() const {
        return default_pitch_;
    }

    void set_pitch(const uint8_t pitch) {
        default_pitch_ = pitch;
    }

    bool is_playing();

private:
    static constexpr uint32_t SAMPLING_FREQ { 44100U * 1U }; /**< DAC Sampling frequency in Hz */
    static constexpr size_t TEXT_QUEUE_SIZE { 4 }; /**< Size of text queue in number of elements */
    static constexpr size_t OUT_QUEUE_SIZE { 4 }; /**< Size of out queue in number of elements */
    static constexpr uint16_t TTS_TASK_STACK_SIZE { 1024 }; /**< Size of stack in byte */
    static constexpr uint8_t TTS_TASK_PRIORITY { 2 };
    static constexpr uint8_t pitches_[] { 1, 2, 4, 6, 8, 10, 13, 16 }; // User specified pitch changes

    uint8_t default_pitch_;
    char phonemes_[128];
    int8_t modifier_[sizeof(phonemes_)];
    std::thread* task_handle_;
    std::atomic<bool> task_running_;
    CircularBuffer<std::string*, TEXT_QUEUE_SIZE> text_queue_;
    CircularBuffer<audio_block_t*, OUT_QUEUE_SIZE> out_queue_;
    audio_block_t* buffer_;
    size_t buffer_idx_;
    int16_t last_sample_;


    static constexpr bool is_whitespace(const char c) {
        return (c == 0 || c == ' ' || c == ',' || c == '.' || c == '?' || c == '\'' || c == '!' || c == ':' || c == '/');
    }

    void audio_processing();

    FLASHMEM bool say_text(const char* text);

    FLASHMEM bool say_phonemes(const char* phonemes);

    FLASHMEM bool text_to_phonemes(const char* src, const VOCAB* vocab, char* dest) const;

    FLASHMEM bool phonemes_to_data(const char* textp, const PHONEME* phoneme);

    FLASHMEM size_t copy_token(char token, char* dest, size_t x, const VOCAB* vocab) const;

    void play(uint8_t duration, uint8_t soundNumber);

    uint8_t play_tone(uint8_t soundNum, uint8_t soundPos, uint8_t pitch1, uint8_t pitch2, uint8_t count, uint8_t volume);

    void sound_on();

    void sound_off();

    void sound(const uint8_t sample, const bool single);

    void pause(const uint8_t d);

    void delay(const uint8_t d);

    void commit_buffer();

    void prepare_buffer();

    void interpolate(const int16_t last_sample, const int16_t new_sample, const uint32_t steps, const uint32_t scaling);

    virtual void update() override;
};

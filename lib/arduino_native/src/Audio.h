/*
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    Audio.h
 * @brief   Wrapper aroung Arduino Audio to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include "AudioStream.h"

#include <cstdint>
#include <cstddef>


static inline size_t AudioMemoryUsage() {
    return 0;
}

static inline size_t AudioMemoryUsageMax() {
    return 0;
}

static inline float AudioProcessorUsage() {
    return 0;
}

static inline float AudioProcessorUsageMax() {
    return 0;
}

static inline void AudioMemory(size_t) {}

// FIXME: just a dummy, to be implemented
class AudioPlaySdWav : public AudioStream {
public:
    AudioPlaySdWav() : AudioStream(0, nullptr) {}

    bool play(const char*) {
        return false;
    }

    void stop() {}

    bool isPlaying() {
        return false;
    }
};

class AudioSynthWaveformSine : public AudioStream {
public:
    AudioSynthWaveformSine() : AudioStream(0, nullptr) {}
    void frequency(float) {}
    void phase(float) {}
    void amplitude(float) {}
    virtual void update();
};

class AudioOutputAnalog : public AudioStream {
public:
    AudioOutputAnalog() : AudioStream(0, nullptr) {}
};

class AudioOutputI2S : public AudioStream {
public:
    AudioOutputI2S() : AudioStream(0, nullptr) {}
};

class AudioMixer4 : public AudioStream {
public:
    AudioMixer4() : AudioStream(0, nullptr) {}
    void gain(int, float) {}
};

class AudioConnection {
public:
    AudioConnection(AudioStream&, AudioStream&) {}
    AudioConnection(AudioStream&, uint8_t, AudioStream&, uint8_t) {}
};

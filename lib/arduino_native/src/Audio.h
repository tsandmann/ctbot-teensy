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

class AudioOutputAnalog : public AudioStream {
public:
    AudioOutputAnalog() : AudioStream(0, nullptr) {}
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

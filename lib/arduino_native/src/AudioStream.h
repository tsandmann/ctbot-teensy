#pragma once

#include "arduino_fixed.h"
#include <cstdint>


#define AUDIO_BLOCK_SAMPLES 128

typedef struct audio_block_struct {
    uint8_t ref_count;
    uint8_t reserved1;
    uint16_t memory_pool_index;
    int16_t data[AUDIO_BLOCK_SAMPLES];
} audio_block_t;

class AudioStream {
public:
    AudioStream(unsigned char, audio_block_t**) {}

    virtual void update() {}

    audio_block_t* allocate() {
        static audio_block_t dummy;
        return &dummy;
    }

    void transmit(audio_block_t*, unsigned char = 0) {}

    void release(audio_block_t*) {}
};

using namespace arduino;
using boolean = bool;

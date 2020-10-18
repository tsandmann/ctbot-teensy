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
 * @file    AudioStream.h
 * @brief   Wrapper aroung Arduino AudioStream to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include "arduino.h"
#include <cstdint>


#define AUDIO_BLOCK_SAMPLES 128

typedef struct audio_block_struct {
    uint8_t ref_count;
    uint8_t reserved1;
    uint16_t memory_pool_index;
    int16_t data[AUDIO_BLOCK_SAMPLES];
} audio_block_t;

// FIXME: just a dummy, to be implemented
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

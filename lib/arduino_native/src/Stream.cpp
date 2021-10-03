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
 * @file    Stream.cpp
 * @brief   Wrapper aroung Arduino Stream library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#include "Stream.h"


void Stream::setRX(uint8_t) {}

void Stream::setTX(uint8_t) {}

void Stream::begin(uint32_t) {}

size_t Stream::readBytes(char* buffer, size_t length) {
    if (buffer == nullptr) {
        return 0;
    }
    size_t count = 0;
    while (count < length) {
        int c = read();
        if (c < 0) {
            break;
        }
        *buffer++ = (char) c;
        count++;
    }
    return count;
}

size_t Stream::readBytesUntil(char terminator, char* buffer, size_t length) {
    if (buffer == nullptr) {
        return 0;
    }
    size_t count = 0;
    while (count < length) {
        int c = read();
        if (c < 0 || c == terminator) {
            break;
        }
        *buffer++ = (char) c;
        count++;
    }
    return count;
}

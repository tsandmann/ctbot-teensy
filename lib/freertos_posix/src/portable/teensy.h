/*
 * This file is part of the FreeRTOS Wrapper for POSIX environments.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    teensy.h
 * @brief   Wrapper aroung FreeRTOS API to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include "arduino_fixed.h"

#include <cstdint>


extern "C" {
/**
 * @brief Write every character from the null-terminated C-string str and one additional newline character '\n' to Serial
 * @param[in] str: Character C-string to be written
 */
void serial_puts(const char* str);
} // extern C

namespace freertos {
/**
 * @brief Indicate an error with the onboard LED
 * @param[in] n: Number of short LED pulses to encode the error
 */
void error_blink(uint8_t number);

/**
 * @brief Get amount of free (heap) RAM
 * @return Free RAM on heap in byte
 */
long free_ram();

/**
 * @brief Print amount of free (heap) RAM to Serial
 */
void print_free_ram();

/**
 * @brief Get the current time in microseconds
 * @return Current time in us
 */
static inline uint32_t get_us() {
    return arduino::micros();
}

/**
 * @brief Get the current time in milliseconds
 * @return Current time in ms
 */
static inline uint32_t get_ms() {
    return arduino::millis();
}

/**
 * @brief Initialize the Sysview interface
 */
static inline void sysview_init() {}

/**
 * @brief Trace an ISR entry
 */
static inline void trace_isr_enter() {}

/**
 * @brief Trace an ISR exit
 */
static inline void trace_isr_exit() {}
} // namespace freertos


namespace freertos {
std::tuple<size_t, size_t, size_t> ram_usage();

void print_ram_usage();
} // namespace freertos

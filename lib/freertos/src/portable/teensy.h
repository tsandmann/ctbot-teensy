/*
 * This file is part of the FreeRTOS port to Teensy boards.
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
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 3
 * @author  Timo Sandmann
 * @date    26.05.2018
 */

#pragma once

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>
#include <tuple>


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
void error_blink(const uint8_t n) __attribute__((noreturn));

/**
 * @brief Get amount of used and free (heap) RAM
 * @return Tuple of: free RAM in byte, used heap in byte, system free in byte
 */
std::tuple<size_t, size_t, size_t> ram_usage();

/**
 * @brief Print amount of used and free (heap) RAM to Serial
 */
void print_ram_usage();

/**
 * @brief Get the current time in microseconds
 * @return Current time in us
 */
uint64_t get_us();

/**
 * @brief Get the current time in milliseconds
 * @return Current time in ms
 */
static inline uint32_t get_ms() {
    return ::xTaskGetTickCount() / (configTICK_RATE_HZ / 1000U);
}

/**
 * @brief Initialize the Sysview interface
 */
void sysview_init();

/**
 * @brief Trace an ISR entry
 */
static inline void trace_isr_enter() {
    traceISR_ENTER();
}

/**
 * @brief Trace an ISR exit
 */
static inline void trace_isr_exit() {
    traceISR_EXIT();
}
} // namespace freertos

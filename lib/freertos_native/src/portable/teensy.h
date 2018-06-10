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

#ifndef PORTABLE_TEENSY_NATIVE_H_
#define PORTABLE_TEENSY_NATIVE_H_

#include <cstdint>


extern "C" uint8_t* stack_top; /**< Pointer to top of initial stack, initialized in setup() */

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
} // namespace freertos

#endif /* PORTABLE_TEENSY_NATIVE_H_ */

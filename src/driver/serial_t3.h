/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    serial_t3.h
 * @brief   Teensy 3.5/3.6 serialport FreeRTOS driver
 * @author  Timo Sandmann
 * @date    16.10.2021
 */

#pragma once

#if defined ARDUINO_TEENSY35 || defined ARDUINO_TEENSY36
#include "serial_io.h"


namespace arduino {
namespace teensy3 {
extern SerialIOStreamAdapter Serial1;
extern SerialIOStreamAdapter Serial2;
extern SerialIOStreamAdapter Serial3;
extern SerialIOStreamAdapter Serial4;
extern SerialIOStreamAdapter Serial5;
extern SerialIOStreamAdapter Serial6;
extern SerialIOStreamAdapter Serial;
} // namespace teensy3

/**
 * @brief Get the serial port object
 * @param[in] port: Number of serial port
 * @return Reference to SerialIO
 */
static inline constexpr SerialIO& get_serial(const uint8_t port) {
    if (port == 1) {
        return arduino::teensy3::Serial1;
    } else if (port == 2) {
        return arduino::teensy3::Serial2;
    } else if (port == 3) {
        return arduino::teensy3::Serial3;
    } else if (port == 4) {
        return arduino::teensy3::Serial4;
    } else if (port == 5) {
        return arduino::teensy3::Serial5;
    } else if (port == 6) {
        return arduino::teensy3::Serial6;
    } else {
        return arduino::teensy3::Serial;
    }
}
} // namespace arduino

#endif // ARDUINO_TEENSY35 || ARDUINO_TEENSY36

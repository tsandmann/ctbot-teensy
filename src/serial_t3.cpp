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
 * @file    serial_t3.cpp
 * @brief   Teensy 3.5/3.6 serialport FreeRTOS driver
 * @author  Timo Sandmann
 * @date    16.10.2021
 */

#if defined ARDUINO_TEENSY35 || defined ARDUINO_TEENSY36
#include "serial_t3.h"


namespace arduino {
namespace teensy3 {
SerialIOStreamAdapter Serial1 { arduino::Serial1 };
SerialIOStreamAdapter Serial2 { arduino::Serial2 };
SerialIOStreamAdapter Serial3 { arduino::Serial3 };
SerialIOStreamAdapter Serial4 { arduino::Serial4 };
SerialIOStreamAdapter Serial5 { arduino::Serial5 };
SerialIOStreamAdapter Serial6 { arduino::Serial6 };
SerialIOStreamAdapter Serial { arduino::Serial };
} // namespace teensy3
} // namespace arduino

#endif // ARDUINO_TEENSY35 || ARDUINO_TEENSY36

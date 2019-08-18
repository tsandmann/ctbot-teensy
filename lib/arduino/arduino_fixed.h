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
 * @file    arduino_fixed.h
 * @brief   Collection of workarounds and fixes to avoid some annoying stuff of Arduino.h
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "Arduino.h"
#include "Print.h"
#include "Wire.h"
#include "SPI.h"

/* get rid of these stupid macros... */
#undef word
#undef F
#undef min
#undef max
#undef abs
#undef constrain
#undef round
#undef radians
#undef degrees
#undef sq
#undef stricmp
#undef sei
#undef cli
#undef interrupts
#undef noInterrupts
#undef clockCyclesPerMicrosecond
#undef clockCyclesToMicroseconds
#undef microsecondsToClockCycles
#undef lowByte
#undef highByte
#undef bitRead
#undef bitSet
#undef bitClear
#undef bitWrite
#undef bit
#undef false
#undef true
#undef BIN
#undef OCT
#undef DEC
#undef HEX
#undef BYTE

#undef HIGH
#undef LOW
#undef INPUT
#undef OUTPUT
#undef INPUT_PULLUP
#undef INPUT_PULLDOWN
#undef OUTPUT_OPENDRAIN
#undef INPUT_DISABLE
#undef LSBFIRST
#undef MSBFIRST
#undef _BV
#undef CHANGE
#undef FALLING
#undef RISING
#undef digitalPinHasPWM
#undef LED_BUILTIN

namespace arduino {
using ::analogRead;
using ::analogReadAveraging;
using ::analogReadResolution;
using ::analogWrite;
using ::analogWriteFrequency;
using ::analogWriteResolution;
using ::attachInterrupt;
using ::digitalReadFast;
using ::digitalWriteFast;
using ::pinMode;

using ::delay;
using ::delayMicroseconds;
using ::micros;
using ::millis;
using ::yield;

using ::HardwareSerial;
using ::Serial;
using ::Serial1;
using ::Serial2;
using ::Serial3;
using ::Serial4;
using ::Serial5;
using ::Serial6;
using ::SPI;
using ::SPI1;
using ::SPI2;
using ::Stream;
using ::TwoWire;
using ::Wire;
using ::Wire1;
using ::Wire2;
// using ::Wire3; //FIXME: activate I2C_3

using ::String;

template <typename T, typename std::enable_if_t<std::is_integral<T>::value, int> = 0>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T, typename std::enable_if_t<std::is_reference<T>::value, int> = 0,
    typename std::enable_if_t<std::is_integral<typename std::remove_reference<T>::type>::value, int> = 0>
T map(const T x, const T in_min, const T in_max, const T out_min, const T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static constexpr uint8_t INPUT { 0 };
static constexpr uint8_t OUTPUT { 1 };
static constexpr uint8_t INPUT_PULLUP { 2 };
static constexpr uint8_t INPUT_PULLDOWN { 3 };
static constexpr uint8_t OUTPUT_OPENDRAIN { 4 };
static constexpr uint8_t INPUT_DISABLE { 5 };

static constexpr uint8_t LED_BUILTIN { 13 };

static constexpr uint8_t FALLING { 2 };
static constexpr uint8_t RISING { 3 };
static constexpr uint8_t CHANGE { 4 };

static constexpr bool digitalPinHasPWM(uint8_t p) {
    return (((p) >= 2 && (p) <= 10) || (p) == 14 || ((p) >= 20 && (p) <= 23) || (p) == 29 || (p) == 30 || ((p) >= 35 && (p) <= 38));
}
} // namespace arduino

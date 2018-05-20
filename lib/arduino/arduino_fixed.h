/*
 * This file is part of the c't-Bot teensy framework.
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

#ifndef _ARDUINO_FIXED_H_
#define _ARDUINO_FIXED_H_

#include <Arduino.h>
#include <Print.h>

namespace arduino {
    using ::analogRead;
    using ::analogReadAveraging;
    using ::analogWrite;
    using ::analogWriteFrequency;
    using ::analogWriteResolution;
    using ::attachInterrupt;
    using ::digitalReadFast;
    using ::digitalWriteFast;
    using ::pinMode;

    using ::delay;
    using ::delayMicroseconds;

    using ::Serial;
}

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

#endif /* _ARDUINO_FIXED_H_ */

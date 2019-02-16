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
 * @file    arm_kinetis_debug.cpp
 * @brief   Wrapper aroung ARMDebug/ARMKinetisDebug to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    21.10.2018
 */

#include "arm_kinetis_debug.h"


bool ARMDebug::begin() {
    return false;
}


ARMKinetisDebug::ARMKinetisDebug(const uint8_t, const uint8_t, const LogLevel) {}

bool ARMKinetisDebug::detect() {
    return false;
}

bool ARMKinetisDebug::reset(const bool) {
    return false;
}

bool ARMKinetisDebug::sys_reset_request() {
    return false;
}

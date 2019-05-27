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
 * @file    arm_kinetis_debug.h
 * @brief   Wrapper aroung ARMDebug/ARMKinetisDebug to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    21.10.2018
 */

#pragma once

#include <cstdint>


class ARMDebug {
public:
    enum LogLevel { LOG_NONE = 0, LOG_ERROR, LOG_NORMAL, LOG_TRACE_MEM, LOG_TRACE_AP, LOG_TRACE_DP, LOG_TRACE_SWD, LOG_MAX };

    bool begin();
};

class ARMKinetisDebug : public ARMDebug {
public:
    ARMKinetisDebug(const uint8_t clockPin, const uint8_t dataPin, const LogLevel logLevel = LOG_NORMAL);

    bool detect();
    bool reset(const bool check_sec_bit = true);
    bool sys_reset_request();
};

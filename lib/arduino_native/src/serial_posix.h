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
 * @file    serial_posix.h
 * @brief   Posix serialport dummy
 * @author  Timo Sandmann
 * @date    21.10.2021
 */

#pragma once

#include "serial_io.h"
#include "ctbot_config.h"


#ifdef CTBOT_SIMULATION
namespace arduino {
namespace posix {
extern SerialIOStreamAdapter Serial;
extern SerialIOStreamAdapter Serial1;
extern SerialIOStreamAdapter Serial2;
extern SerialIOStreamAdapter Serial3;
extern SerialIOStreamAdapter Serial4;
extern SerialIOStreamAdapter Serial5;
extern SerialIOStreamAdapter Serial6;
extern SerialIOStreamAdapter Serial7;
extern SerialIOStreamAdapter Serial8;
} // namespace posix

static inline constexpr SerialIO& get_serial(const uint8_t port) {
    if (port == ctbot::CtBotConfig::UART_FOR_CMD) {
        return arduino::posix::Serial;
    }

    if (port == 1) {
        return arduino::posix::Serial1;
    } else if (port == 2) {
        return arduino::posix::Serial2;
    } else if (port == 3) {
        return arduino::posix::Serial3;
    } else if (port == 4) {
        return arduino::posix::Serial4;
    } else if (port == 5) {
        return arduino::posix::Serial5;
    } else if (port == 6) {
        return arduino::posix::Serial6;
    } else if (port == 7) {
        return arduino::posix::Serial7;
    } else if (port == 8) {
        return arduino::posix::Serial8;
    }

    return arduino::posix::Serial;
}
} // namespace arduino
#endif // CTBOT_SIMULATION

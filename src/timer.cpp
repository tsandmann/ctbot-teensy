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
 * @file    timer.cpp
 * @brief   Timer helper functions
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "timer.h"

#include "arduino_freertos.h"

#include <chrono>


namespace ctbot {

__attribute__((weak)) uint32_t Timer::get_us() {
    return static_cast<uint32_t>(freertos::get_us());
}

__attribute__((weak)) uint32_t Timer::get_us_from_isr() {
    return static_cast<uint32_t>(freertos::get_us_from_isr());
}

__attribute__((weak)) uint32_t Timer::get_ms() {
    using namespace std::chrono;
    return static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
}

__attribute__((weak)) void Timer::delay_us(const uint32_t us) {
    arduino::delayMicroseconds(us);
}

} // namespace ctbot

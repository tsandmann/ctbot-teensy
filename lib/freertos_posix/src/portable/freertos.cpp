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
 * @file    freertos.cpp
 * @brief   Wrapper aroung FreeRTOS API to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#include "FreeRTOS.h"
#include "arduino_fixed.h"
#include "teensy.h"

#include <tuple>
#include <iostream>
#include <cstdio>
#include <thread>
#include <chrono>


extern "C" {
void serial_puts(const char* str) {
    ::puts(str);
}

#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook();
void vApplicationIdleHook() {
    std::this_thread::sleep_for(std::chrono::microseconds(10));
}
#endif // configUSE_IDLE_HOOK
} // extern C

namespace freertos {
std::tuple<size_t, size_t, size_t> ram_usage() {
    const std::tuple<size_t, size_t, size_t> ret { 0xffffff, 0, 0 };
    return ret;
}

void print_ram_usage() {
    const auto x { ram_usage() };
    std::cout << "free RAM: " << std::get<0>(x) / 1024UL << " KB\n";
}

void error_blink(uint8_t) {}
} // namespace freertos

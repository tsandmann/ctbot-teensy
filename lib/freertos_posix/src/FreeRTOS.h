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
 * @file    FreeRTOS.h
 * @brief   Wrapper aroung FreeRTOS API to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include "task.h"
#include <cassert>
#include <cstdint>
#include <thread>

static constexpr unsigned long tskIDLE_PRIORITY { 1 };

namespace free_rtos_std {
class gthr_freertos {
public:
    using native_task_type = void*;

    static StackType_t set_next_stacksize(StackType_t /*size*/) {
        return 0;
    }

    static void set_priority(native_task_type /*task*/, const uint32_t /*prio*/) {}

    static void set_name(native_task_type /*task*/, const char* /*task_name*/) {}

    static void suspend(std::thread*) {}

    static void resume(std::thread*) {}

    static void* get_freertos_handle(std::thread*) {
        return nullptr;
    }
};

} // namespace free_rtos_std

extern "C" {
void vTaskStartScheduler();
void vTaskEndScheduler();
void assert_blink(const char*, int, const char*, const char*);
} // extern C

#define configMAX_TASK_NAME_LEN 10
#define configIDLE_TASK_NAME "idle"
#define portMAX_DELAY (TickType_t) 0xffffffffUL

#define portYIELD_FROM_ISR(x) ((void) 0)

#ifdef NDEBUG
#define configASSERT(condition) ((void) 0)
#else
#define configASSERT(__e) ((__e) ? (void) 0 : assert_blink(__FILE__, __LINE__, __PRETTY_FUNCTION__, #__e), assert(__e))
#endif // NDEBUG

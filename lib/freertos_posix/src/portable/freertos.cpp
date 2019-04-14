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

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <thread>
#include <chrono>
#include <tuple>


extern "C" {
void serial_puts(const char* str) {
    ::puts(str);
}

void vTaskStartScheduler() {
    while (true) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1s);
    }
}

void vTaskEndScheduler() {
    ::exit(0);
}

void vTaskSuspendAll() {}

long xTaskResumeAll() {
    return 0;
}

void vTaskResume(TaskHandle_t) {}

void vTaskSuspend(TaskHandle_t) {}

void vTaskDelete(TaskHandle_t) {}

void vTaskPrioritySet(TaskHandle_t, UBaseType_t) {}

BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t*, TickType_t) {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
    return 0;
}

BaseType_t xTaskNotifyFromISR(TaskHandle_t, uint32_t, eNotifyAction, BaseType_t*) {
    return 1;
}

TaskHandle_t xTaskGetHandle(const char*) {
    return nullptr; // FIXME: check this
}

char* pcTaskGetName(TaskHandle_t) {
    static char dummy[configMAX_TASK_NAME_LEN + 1] { "INVALID" };
    return dummy;
}

TaskHandle_t xTaskGetIdleTaskHandle() {
    return nullptr; // FIXME: check this
}

UBaseType_t uxTaskPriorityGet(TaskHandle_t) {
    return 0;
}

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t) {
    return 0xffffff / sizeof(StackType_t);
}

UBaseType_t uxTaskGetNumberOfTasks() {
    return 0; // FIXME: implement?
}

UBaseType_t uxTaskGetSystemState(TaskStatus_t* const /*pxTaskStatusArray*/, const UBaseType_t /*uxArraySize*/, uint32_t* const /*pulTotalRunTime*/) {
    return 0; // FIXME: implement?
}

void vApplicationIdleHook();
void vApplicationIdleHook() {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);
}

void assert_blink(const char* file, int line, const char* func, const char* expr) {
    std::cout << file << " (" << line << "):" << func << " - " << expr << "\n";
}
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

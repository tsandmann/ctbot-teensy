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
#include "task.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <iostream>


static std::recursive_timed_mutex g_mutex;
static std::vector<std::thread*> g_task_list;

uint8_t* stack_top { nullptr }; /**< Pointer to top of (initial) stack, necessary for _sbrk() */

uint32_t xTaskCreate(std::function<void(void*)> pvTaskCode, const char* const, unsigned short, void* pvParameters, uint32_t, void** pxCreatedTask) {
    auto p_thread { new std::thread(pvTaskCode, pvParameters) };
    g_mutex.lock();
    g_task_list.push_back(p_thread);
    g_mutex.unlock();

    if (pxCreatedTask) {
        *pxCreatedTask = p_thread;
    }
    return p_thread != nullptr;
}

void vTaskDelay(const uint32_t ticks) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
}

void vTaskSuspend(void* task_handle) {
    // FIXME: how to implement this?
}

void vTaskResume(void* task_hanlde) {
    // FIXME: how to implement this?
}

void vTaskSuspendAll() {
    g_mutex.lock();
}

long xTaskResumeAll() {
    g_mutex.unlock();
    return 0;
}

void vTaskPrioritySet(void*, uint32_t) {}

void vTaskStartScheduler() {
    vTaskDelay(5000UL);


    if (g_task_list.size() >= 2) {
        auto task { g_task_list[1] }; // main task
        if (task->joinable()) {
            task->join();
        }
    }

    for (const auto& task : g_task_list) {
        if (task->joinable()) {
            task->join();
        }
    }
}

void vTaskEndScheduler() {
    std::cout << "vTaskEndScheduler() called. Exiting.\n";
    vTaskSuspendAll();
    xTaskResumeAll();
    exit(0);
}

void* xTaskGetIdleTaskHandle() {
    return nullptr;
}

uint32_t uxTaskGetStackHighWaterMark(void*) {
    return 0xffff;
}

void* xSemaphoreCreateMutex() {
    return new std::recursive_timed_mutex;
}

long xSemaphoreTake(void* mutex, uint32_t max_delay) {
    if (mutex) {
        std::recursive_timed_mutex* p_mtx { reinterpret_cast<std::recursive_timed_mutex*>(mutex) };
        if (max_delay == portMAX_DELAY) {
            p_mtx->lock();
            return 1;
        } else if (p_mtx->try_lock_for(std::chrono::milliseconds(max_delay))) { // we assume a tick rate of 1 kHz here
            return 1;
        }
    }

    return 0;
}

long xSemaphoreGive(void* mutex) {
    if (mutex) {
        std::recursive_timed_mutex* p_mtx { reinterpret_cast<std::recursive_timed_mutex*>(mutex) };
        p_mtx->unlock();
        return 1;
    }

    return 0;
}

namespace freertos {
long free_ram() {
    return 0xffffff;
}

void print_free_ram() {
    std::cout << "free RAM: " << freertos::free_ram() / 1024UL << " KB\n";
}

void error_blink(uint8_t) {}
} // namespace freertos

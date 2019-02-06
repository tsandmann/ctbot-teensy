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
#include "arduino_fixed.h"

#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <cassert>


static std::recursive_timed_mutex g_mutex;
static std::vector<std::thread*> g_task_list;
static pthread_mutex_t g_suspend_resume_mutex = PTHREAD_MUTEX_INITIALIZER;

extern "C" {
void serial_puts(const char* str) {
    ::puts(str);
}
} // extern C

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

void vTaskDelete(void* task_handle) {
    g_mutex.lock();
    for (auto it = g_task_list.begin(); it != g_task_list.end(); ++it) {
        if (*it == task_handle) {
            g_task_list.erase(it);
            break;
        }
    }
    g_mutex.unlock();
    if (task_handle) {
        std::thread* p_thread { reinterpret_cast<std::thread*>(task_handle) };
        p_thread->detach();
    }
}

uint32_t xTaskGetTickCount() {
    return arduino::millis();
}

void vTaskDelay(const uint32_t ticks) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
}

void vTaskSuspend(void* task_handle) {
    pthread_t thread;
    if (task_handle) {
        std::thread* p_thread { reinterpret_cast<std::thread*>(task_handle) };
        thread = p_thread->native_handle();
    } else {
        thread = pthread_self();
    }

    const int rc { pthread_mutex_lock(&g_suspend_resume_mutex) };
    assert(rc == 0);
    pthread_kill(thread, SIGALRM);
}

/**
 *  Signal handler for SIGALRM (suspend)
 */
static void SuspendSignalHandler(int sig) {
    std::cout << "SuspendSignalHandler() called.\n";
    /* we are only interested in the resume signal */
    sigset_t signals;
    sigemptyset(&signals);
    sigaddset(&signals, SIGVTALRM /*resume*/);

    /* wait on the resume signal... */
    pthread_mutex_unlock(&g_suspend_resume_mutex); // FIXME: should be atomic
    const int rc { sigwait(&signals, &sig) };
    assert(rc == 0);
    std::cout << "task resumed.\n";
}

void vTaskResume(void* task_handle) {
    pthread_t thread;
    if (task_handle) {
        std::thread* p_thread { reinterpret_cast<std::thread*>(task_handle) };
        thread = p_thread->native_handle();
    } else {
        thread = pthread_self();
    }

    const int rc { pthread_mutex_lock(&g_suspend_resume_mutex) };
    assert(rc == 0);

    if (!pthread_equal(pthread_self(), thread)) {
        pthread_kill(thread, SIGVTALRM);
    }
    pthread_mutex_unlock(&g_suspend_resume_mutex);
}

/**
 *  Signal handler for SIGVTALRM (resume)
 */
static void ResumeSignalHandler(int) {}

void vTaskSuspendAll() {
    g_mutex.lock();
}

long xTaskResumeAll() {
    g_mutex.unlock();
    return 0;
}

void vTaskPrioritySet(void*, uint32_t) {}

void vTaskStartScheduler() {
    struct sigaction sigsuspend;
    memset(&sigsuspend, 0, sizeof(sigsuspend));
    sigemptyset(&sigsuspend.sa_mask);
    sigsuspend.sa_flags = 0;
    sigsuspend.sa_handler = SuspendSignalHandler;
    if (sigaction(SIGALRM, &sigsuspend, nullptr)) {
        std::cerr << "Problem installing SIGALRM\n";
    } else {
        std::cout << "vTaskStartScheduler(): SIGALRM installed.\n";
    }

    struct sigaction sigresume;
    memset(&sigresume, 0, sizeof(sigresume));
    sigemptyset(&sigresume.sa_mask);
    sigresume.sa_flags = 0;
    sigresume.sa_handler = ResumeSignalHandler;
    if (sigaction(SIGVTALRM, &sigresume, nullptr)) {
        std::cerr << "Problem installing SIGVTALRM\n";
    } else {
        std::cout << "vTaskStartScheduler(): SIGVTALRM installed.\n";
    }

    vTaskDelay(2000UL); // FIXME: baeh

    if (g_task_list.size()) {
        std::thread* p_task { g_task_list[0] }; // main task
        if (p_task->joinable()) {
            p_task->join();
        }
    }

    for (const auto& p_task : g_task_list) {
        if (p_task->joinable()) {
            p_task->join();
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

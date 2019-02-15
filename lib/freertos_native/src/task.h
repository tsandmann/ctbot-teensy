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
 * @file    task.h
 * @brief   Wrapper aroung FreeRTOS API to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include <cstdint>
#include <functional>


static constexpr uint32_t tskIDLE_PRIORITY { 0 };
static constexpr char configIDLE_TASK_NAME[] { "IDLE" };

typedef void* TaskHandle_t;

extern "C" {
typedef enum {
    eNoAction = 0, /* Notify the task without updating its notify value. */
    eSetBits, /* Set bits in the task's notification value. */
    eIncrement, /* Increment the task's notification value. */
    eSetValueWithOverwrite, /* Set the task's notification value to a specific value even if the previous value has not yet been read by the task. */
    eSetValueWithoutOverwrite /* Set the task's notification value if the previous value has been read by the task. */
} eNotifyAction;

uint32_t xTaskCreate(std::function<void(void*)> pvTaskCode, const char* const pcName, unsigned short usStackDepth, void* pvParameters, uint32_t uxPriority,
    void** pxCreatedTask);

void vTaskDelete(void* task_handle);

void vTaskDelay(const uint32_t ticks);

uint32_t xTaskGetTickCount();

void vTaskSuspend(void* task_handle);

void vTaskResume(void* task_handle);

void vTaskPrioritySet(void* task_handle, uint32_t prio);

void vTaskSuspendAll();

long xTaskResumeAll();

long xTaskNotifyWait(uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t* pulNotificationValue, uint32_t xTicksToWait);

long xTaskGenericNotifyFromISR(
    void* xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t* pulPreviousNotificationValue, long* pxHigherPriorityTaskWoken);

static inline long xTaskNotifyFromISR(void* xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, long* pxHigherPriorityTaskWoken) {
    return xTaskGenericNotifyFromISR(xTaskToNotify, ulValue, eAction, nullptr, pxHigherPriorityTaskWoken);
}

void* xTaskGetHandle(const char* pcNameToQuery);

char* pcTaskGetName(void* xTaskToQuery);

long uxTaskPriorityGet(void* task_handle);
} // extern C

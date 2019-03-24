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


typedef void* TaskHandle_t;
typedef unsigned StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;
typedef uint32_t TickType_t;

typedef enum {
    eNoAction = 0, /* Notify the task without updating its notify value. */
    eSetBits, /* Set bits in the task's notification value. */
    eIncrement, /* Increment the task's notification value. */
    eSetValueWithOverwrite, /* Set the task's notification value to a specific value even if the previous value has not yet been read by the task. */
    eSetValueWithoutOverwrite /* Set the task's notification value if the previous value has been read by the task. */
} eNotifyAction;

extern "C" {
void vTaskSuspendAll();

long xTaskResumeAll();

void vTaskResume(TaskHandle_t xTaskToResume);

void vTaskSuspend(TaskHandle_t xTaskToSuspend);

void vTaskDelete(TaskHandle_t xTaskToDelete);

void vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority);

BaseType_t xTaskNotifyWait(uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t* pulNotificationValue, TickType_t xTicksToWait);

BaseType_t xTaskNotifyFromISR(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, BaseType_t* pxHigherPriorityTaskWoken);

TaskHandle_t xTaskGetHandle(const char* pcNameToQuery);

UBaseType_t uxTaskPriorityGet(TaskHandle_t xTask);

char* pcTaskGetName(TaskHandle_t xTaskToQuery);

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask);

void* xTaskGetIdleTaskHandle();
} // extern C

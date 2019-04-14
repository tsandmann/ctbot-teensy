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

typedef enum {
    eRunning = 0, /* A task is querying the state of itself, so must be running. */
    eReady, /* The task being queried is in a read or pending ready list. */
    eBlocked, /* The task being queried is in the Blocked state. */
    eSuspended, /* The task being queried is in the Suspended state, or is in the Blocked state with an infinite time out. */
    eDeleted, /* The task being queried has been deleted, but its TCB has not yet been freed. */
    eInvalid /* Used as an 'invalid state' value. */
} eTaskState;

typedef struct xTASK_STATUS {
    TaskHandle_t xHandle; /* The handle of the task to which the rest of the information in the structure relates. */
    const char* pcTaskName; /* A pointer to the task's name. */
    UBaseType_t xTaskNumber; /* A number unique to the task. */
    eTaskState eCurrentState; /* The state in which the task existed when the structure was populated. */
    UBaseType_t uxCurrentPriority; /* The priority at which the task was running (may be inherited) when the structure was populated. */
    UBaseType_t uxBasePriority; /* The priority to which the task will return if the task's current priority has been inherited to avoid unbounded priority
                                   inversion when obtaining a mutex. */
    uint32_t ulRunTimeCounter; /* The total run time allocated to the task so far, as defined by the run time stats clock. */
    StackType_t* pxStackBase; /* Points to the lowest address of the task's stack area. */
    uint16_t usStackHighWaterMark; /* The minimum amount of stack space that has remained for the task since the task was created.  The closer this value is to
                                      zero the closer the task has come to overflowing its stack. */
} TaskStatus_t;

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

UBaseType_t uxTaskGetNumberOfTasks();

UBaseType_t uxTaskGetSystemState(TaskStatus_t* const pxTaskStatusArray, const UBaseType_t uxArraySize, uint32_t* const pulTotalRunTime);
} // extern C

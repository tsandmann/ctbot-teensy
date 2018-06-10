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

#ifndef _FREERTOS_NATIVE_H_
#define _FREERTOS_NATIVE_H_

#include <cstdint>


using StackType_t = uint32_t;

static constexpr uint32_t configTICK_RATE_HZ { 1000UL };
static constexpr uint32_t portMAX_DELAY { 0xffff };
static constexpr uint8_t configMAX_PRIORITIES { 8 };

extern "C" {
void vTaskStartScheduler();

void vTaskEndScheduler();

void* xTaskGetIdleTaskHandle();

uint32_t uxTaskGetStackHighWaterMark(void*);
} // extern C

#endif /* _FREERTOS_NATIVE_H_ */

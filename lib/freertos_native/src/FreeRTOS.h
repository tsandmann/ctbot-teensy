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

#include <cstdint>
#include <cassert>
#include <tuple>


using StackType_t = uint32_t;

static constexpr uint32_t configTICK_RATE_HZ { 1000UL };
static constexpr uint32_t portMAX_DELAY { 0xffff };
static constexpr uint8_t configMAX_PRIORITIES { 8 };
static constexpr uint32_t configCPU_CLOCK_HZ { 180000000UL };

namespace freertos {
std::tuple<size_t, size_t, size_t> ram_usage();

void print_ram_usage();
} // namespace freertos

extern "C" {
void vTaskStartScheduler();

void vTaskEndScheduler();

void* xTaskGetIdleTaskHandle();

uint32_t uxTaskGetStackHighWaterMark(void*);

void portYIELD_FROM_ISR(uint8_t);
} // extern C

#ifdef NDEBUG
#define configASSERT(condition) ((void) 0)
#else
#define configASSERT(__e) (assert(__e))
#endif

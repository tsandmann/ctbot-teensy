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
 * @file    queue.h
 * @brief   Wrapper aroung FreeRTOS API to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include <cstdint>


extern "C" {
void* xQueueGenericCreate(const long uxQueueLength, const long uxItemSize, const uint8_t ucQueueType);

static inline void* xQueueCreate(const long uxQueueLength, const long uxItemSize) {
    return xQueueGenericCreate(uxQueueLength, uxItemSize, 0);
}

long xQueueGenericSend(void* xQueue, const void* const pvItemToQueue, long xTicksToWait, const long xCopyPosition);

static inline long xQueueSendToBack(void* xQueue, const void* const pvItemToQueue, uint32_t xTicksToWait) {
    return xQueueGenericSend(xQueue, pvItemToQueue, xTicksToWait, 0);
}

void vQueueDelete(void* xQueue);

long uxQueueMessagesWaiting(const void* xQueue);

long xQueueReceive(void* xQueue, void* const pvBuffer, uint32_t xTicksToWait);

} // extern C

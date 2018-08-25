/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    timer.cpp
 * @brief   Timer helper functions
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "timer.h"

#include <arduino_fixed.h>
#include <FreeRTOS.h>
#include <task.h>
#include <util/atomic.h>


namespace ctbot {

uint32_t Timer::get_us() {
    uint32_t current, load, count, istatus;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current = SYST_CVR;
        load = SYST_RVR;
        count = get_ms();
        istatus = SCB_ICSR; // bit 26 indicates if systick exception pending
    }

    if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) {
        ++count;
    }

    current = load - current;
    return count * 1000U + current / (configCPU_CLOCK_HZ / 1000000U);
}

uint32_t Timer::get_ms() {
    return xTaskGetTickCount() / (configTICK_RATE_HZ / 1000U);
}

void Timer::delay_ms(const uint32_t ms) {
    ::vTaskDelay(ms * (configTICK_RATE_HZ / 1000U));
}

void Timer::delay_us(const uint32_t us) {
    arduino::delayMicroseconds(us);
}

} // namespace ctbot

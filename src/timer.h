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
 * @file    timer.h
 * @brief   Timer helper functions
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include "ctbot_config.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Class to group timer helper functions
 *
 * @startuml{Timer.png}
 *   class Timer {
 *     +{static} get_us(T&) : uint32_t
 *     +{static} get_us() : uint32_t
 *     +{static} get_ms() : uint32_t
 *   }
 * @enduml
 */
class Timer {
public:
    /**
     * @brief Get the current time in microseconds
     * @tparam T: Ignored for teensy version
     * @return Current time in us
     */
    template <typename T = uint32_t>
    static uint32_t get_us(T&) {
        return get_us();
    }

    /**
     * @brief Get the current time in microseconds
     * @return Current time in us
     */
    static uint32_t get_us();

    /**
     * @brief Get the current time in milliseconds
     * @return Current time in ms
     */
    static uint32_t get_ms();

    /**
     * @brief Delay the execution by ms milliseconds
     * @param ms: Number of ms to wait
     * @note Calls Arduinos delay()
     */
    static void delay(uint32_t ms);

    /**
     * @brief Delay the execution by us microseconds
     * @param us: Number of us to wait
     * @note Calls Arduinos delayMicroseconds()
     */
    static void delay_us(uint32_t us);
};

} /* namespace ctbot */

#endif /* SRC_TIMER_H_ */

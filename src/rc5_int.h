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
 * @file    rc5_int.h
 * @brief   Interrupt driven RC5 decoder
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "timer.h"

#include "arduino_freertos.h"

#include <cstdint>


class RC5;

namespace ctbot {

/**
 * @brief Interrupt driven RC5 decoder driver
 *
 * @startuml{Rc5.png}
 *  !include rc5_int.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Rc5 {
protected:
    static constexpr bool DEBUG_ { false }; /**< Flag to enable debug output */

    struct rc5_t {
        uint32_t us;
        bool value;
    } __attribute__((packed));

    uint8_t last_idx_;
    uint32_t last_time_;
    uint8_t rc5_addr_;
    uint8_t rc5_cmd_;
    bool rc5_toggle_;
    RC5* p_impl_;

    void reset();

public:
    static constexpr uint8_t DATA_ARRAY_SIZE { 32 }; /**< Size of buffer array in byte for raw input data */
    static rc5_t input_data_[]; /**< Raw input data buffer for IR receiver signal */
    static volatile uint8_t input_idx_; /**< Current index in input data buffer, pointing to the latest entry */

    /**
     * @brief Construct a new Rc5 object
     * @param[in] pin: Pin number of the input data signal, only used for initialization
     */
    FLASHMEM Rc5(const uint8_t pin);

    /**
     * @brief Destroy the Rc5 object
     */
    FLASHMEM ~Rc5();

    /**
     * @brief Check for new input data and process it, if available
     * @return If a complete RC5 command is received, true is returned
     */
    bool update();

    /**
     * @return Address of last received RC5 data
     */
    auto get_addr() const {
        return rc5_addr_;
    }

    /**
     * @return Command of last received RC5 data
     */
    auto get_cmd() const {
        return rc5_cmd_;
    }

    /**
     * @return Toggle bit of last received RC5 data
     */
    auto get_toggle() const {
        return rc5_toggle_;
    }

    /**
     * @brief Reset last received RC5 data
     */
    void reset_rc5() {
        rc5_addr_ = rc5_cmd_ = 0;
    }

    void set_rc5(const uint8_t addr, const uint8_t cmd);

    void set_rc5(const uint8_t addr, const uint8_t cmd, bool toggle);

    /**
     * @brief ISR for RC5 pin change interrupt
     * @tparam PIN_NUM: Number of interrupt vector (to distinguish ISRs)
     * @tparam ARRAY_SIZE: Size of raw input data array in byte
     */
    template <uint8_t PIN_NUM, uint8_t ARRAY_SIZE>
    static inline __attribute__((always_inline)) void isr(rc5_t* p_data, volatile uint8_t* p_idx) {
        static bool last { true };
        static uint32_t last_time {};

        const auto now { Timer::get_us() };
        const bool value { static_cast<bool>(arduino::digitalReadFast(PIN_NUM)) };

        const auto diff { std::abs(static_cast<int32_t>(now) - static_cast<int32_t>(last_time)) };
        if (value != last && diff >= 400 /*&& (!value || diff <= 2300)*/) { // FIXME: check 2300 condition
            last = value;
            last_time = now;

            uint8_t idx { *p_idx };
            p_data[idx].us = now;
            p_data[idx].value = value;
            ++idx;
            idx %= ARRAY_SIZE;
            *p_idx = idx;
        }
    }
};

} // namespace ctbot

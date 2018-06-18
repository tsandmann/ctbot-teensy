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

#ifndef SRC_RC5_INT_H_
#define SRC_RC5_INT_H_

#include "timer.h"

#include <cstdint>


class RC5;

namespace ctbot {

/**
 * @brief Interrupt driven RC5 decoder driver
 */
class Rc5 {
protected:
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
    Rc5(const uint8_t pin);

    /**
     * @brief Destroy the Rc5 object
     */
    ~Rc5();

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

    /**
     * @brief ISR for RC5 pin change interrupt
     * @tparam VECT_NUM: Number of interrupt vector (to distinguish ISRs)
     * @tparam ARRAY_SIZE: Size of raw input data array in byte
     */
    template <uint8_t VECT_NUM, uint8_t ARRAY_SIZE>
    static inline __attribute__((always_inline)) void isr(const bool value, rc5_t* p_data, volatile uint8_t* p_idx) {
        const auto now { Timer::get_us() };

        uint8_t idx { *p_idx };
        p_data[idx].us = now;
        p_data[idx].value = value;
        ++idx;
        idx %= ARRAY_SIZE;
        *p_idx = idx;
    }
};

} /* namespace ctbot */

#endif /* SRC_RC5_INT_H_ */

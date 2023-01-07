/*
 * This file is part of the ct-Bot teensy framework.
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
 * @file    encoder.h
 * @brief   Wheel encoder driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "ctbot_config.h"
#include "timer.h"

#include "arduino_freertos.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Wheel encoder sensor processing
 *
 * @startuml{Encoder.png}
 *  !include encoder.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Encoder {
protected:
    static constexpr float AVG_FILTER_PARAM { CtBotConfig::ENCODER_MARKS > 100 ? 0.1f : 0.25f }; /**< Filter coefficient for speed averaging filter */
    static constexpr float WHEEL_PERIMETER { 178.1283 }; /**< Perimeter of the wheel in mm */
    static constexpr bool DEBUG_ { false }; /**< Flag to enable debug output */

    int16_t edges_; /**< Current number of edges counted; increasing for forward wheel turning, decreasing otherwise */
    uint8_t last_idx_; /**< Index in input data array of last processed entry */
    float speed_; /**< Current speed of wheel in mm/s */
    float speed_avg_; /**< Current speed of wheel as average in mm/s, @see AVG_FILTER_PARAM */
    bool direction_; /**< Current direction set for wheel turning; true: forward, false: backwards */

    const uint32_t* const p_enc_data_; /**< Pointer to data array to use where the raw input data is stored */
    const volatile uint8_t* const p_enc_idx_; /**< Pointer to current index in data array */
    uint32_t last_update_; /**< Timestamp of last edge processing */
    int8_t count_; /**< Internal counter for number of edges since last update */

public:
    /** Size of buffer array in byte for raw encoder data */
    static constexpr uint8_t DATA_ARRAY_SIZE { CtBotConfig::ENCODER_MARKS <= 60 ? 32 : 64 };

    static constexpr float rpm_to_speed(const float rpm) {
        return rpm * WHEEL_PERIMETER / 60.f / 150.f;
    }

    /**
     * @brief Construct a new Encoder object
     * @param[in] p_data: Pointer to data array to use where the raw input data is stored
     * @param[in] p_idx: Pointer to current index in data array
     * @param[in] pin: Pin number of the input data signal
     */
    FLASHMEM Encoder(uint32_t* p_data, volatile uint8_t* p_idx, const uint8_t pin);

    /**
     * @brief Check for new input data and calculate current speed
     */
    void update();

    /**
     * @return Number of measured edges
     */
    auto get() const {
        return edges_;
    }

    /**
     * @return Current speed average in mm/s
     */
    auto get_speed() const {
        return speed_avg_;
    }

    /**
     * @brief Set direction of wheel turning
     * @param[in] dir: true, if wheel is turning forwards, false otherwise
     */
    void set_direction(const bool dir) {
        direction_ = dir;
    }

    /**
     * @brief ISR for wheel encoder pin change interrupt
     * @tparam PIN_NUM: Number of pin to read from
     * @tparam ARRAY_SIZE: Size of raw input data array in byte
     */
    template <uint8_t PIN_NUM, uint8_t ARRAY_SIZE>
    static inline __attribute__((always_inline)) void isr(uint32_t* p_data, volatile uint8_t* p_idx) {
        static bool last {};
        static uint32_t last_time {};

        const uint32_t now { Timer::get_us_from_isr() };
        const bool value { static_cast<bool>(arduino::digitalReadFast(PIN_NUM)) };

        constexpr int32_t MIN_DT_US { static_cast<int32_t>(1'000'000.f / (200.f * 1.2f / 60.f) / CtBotConfig::ENCODER_MARKS) }; // max 200 rpm +20% margin
        if (value != last && (std::abs(static_cast<int32_t>(now) - static_cast<int32_t>(last_time)) > MIN_DT_US)) {
            last = value;
            last_time = now;

            uint8_t idx { *p_idx };
            ++idx;
            idx %= ARRAY_SIZE;
            *p_idx = idx;
            p_data[idx] = now;
        }
    }
};

} // namespace ctbot

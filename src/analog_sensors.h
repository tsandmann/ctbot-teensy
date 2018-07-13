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
 * @file    analog_sensors.h
 * @brief   Analog sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#ifndef SRC_ANALOG_SENSORS_H_
#define SRC_ANALOG_SENSORS_H_

#include "ctbot_config.h"
#include "ena.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Abstraction layer for (simple) analog sensors
 * @note No further sensor data processing is done here, just the raw ADC values are collected.
 */
class AnalogSensors {
protected:
    static constexpr auto ENA_MASK = EnaTypes::BORDER | EnaTypes::LINE; // | EnaTypes::DISTANCE;

    uint32_t last_dist_update_;
    uint8_t last_adc_res_;
    uint16_t distance_[2];
    uint16_t line_[2];
    uint16_t ldr_[2];
    uint16_t border_[2];

    /**
     * @brief Read all the current ADC values
     */
    void update();

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @return ADC value in 10 bit resolution
     */
    int16_t analog_read(const uint8_t pin) {
        return analog_read(pin, 10, 1);
    }

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @param[in] resolution: Resolution in bit for ADC
     * @return ADC value in 10 bit resolution
     */
    int16_t analog_read(const uint8_t pin, const uint8_t resolution) {
        return analog_read(pin, resolution, 1);
    }

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @param[in] resolution: Resolution in bit for ADC
     * @param[in] avg_num: Number of reads to build an average
     * @return ADC value in selected resolution
     */
    int16_t analog_read(const uint8_t pin, const uint8_t resolution, const uint8_t avg_num);

public:
    /**
     * @brief Construct a new AnalogSensors object
     */
    AnalogSensors();

    /**
     * @return The last value of left border sensor
     */
    auto get_border_l() const {
        return border_[0];
    }

    /**
     * @return The last value of right border sensor
     */
    auto get_border_r() const {
        return border_[1];
    }

    /**
     * @return The last value of left distance sensor
     */
    auto get_distance_l() const {
        return distance_[0];
    }

    /**
     * @return The last value of right distance sensor
     */
    auto get_distance_r() const {
        return distance_[1];
    }

    /**
     * @return The last value of left LDR sensor
     */
    auto get_ldr_l() const {
        return ldr_[0];
    }

    /**
     * @return The last value of right LDR sensor
     */
    auto get_ldr_r() const {
        return ldr_[1];
    }

    /**
     * @return The last value of left line sensor
     */
    auto get_line_l() const {
        return line_[0];
    }

    /**
     * @return The last value of right line sensor
     */
    auto get_line_r() const {
        return line_[1];
    }
};

} // namespace ctbot

#endif /* SRC_ANALOG_SENSORS_H_ */

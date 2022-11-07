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
 * @file    analog_sensors.h
 * @brief   Analog sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "ctbot_config.h"

#include "driver/ena_i2c.h"
#include "driver/leds_i2c.h"

#include <cstdint>
#include <mutex>


namespace ctbot {

/**
 * @brief Abstraction layer for (simple) analog sensors
 * @note No further sensor data processing is done here, just the raw ADC values are collected.
 *
 * @startuml{AnalogSensors.png}
 *  !include analog_sensors.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class AnalogSensors {
    static constexpr bool DEBUG_ { false };

protected:
    static constexpr auto ENA_MASK_INIT = EnaI2cTypes::NONE;
    static constexpr auto ENA_MASK = EnaI2cTypes::NONE;
    static constexpr auto ENA_MASK_PWM = LedTypesEna<>::BORDER_L | LedTypesEna<>::BORDER_R | LedTypesEna<>::LINE_L | LedTypesEna<>::LINE_R;
    static constexpr auto ENA_MASK_PWM_INIT = LedTypesEna<>::NONE;
    static constexpr uint8_t BAT_ADC_RES { 12 };
    static constexpr uint32_t BAT_VOLTAGE_R1 { 100'000 + CtBotConfig::VOLTAGE_MEASURE_OFFSETS[0] };
    static constexpr uint32_t BAT_VOLTAGE_R2 { 22'000 + CtBotConfig::VOLTAGE_MEASURE_OFFSETS[1] };
    static constexpr uint8_t CURRENT_ADC_RES_ { 12 };
    static constexpr float OPAMP_AMPLIFICATION_ { 50.f }; // V/V
    static constexpr float SHUNT_5V_MOHM_ { 30.f }; // mOhm
    static constexpr float SHUNT_SERVO_MOHM_ { 100.f }; // mOhm
    static constexpr float ADC_VREF_MV_ { 3'300.f }; // mV
    static constexpr float CURRENT_MEASUREMENT_ALPHA_ { 0.05f };
    static constexpr float CURRENT_5V_CONVERSION_FACTOR_ =
        ADC_VREF_MV_ / ((1 << CURRENT_ADC_RES_) - 1.f) / (OPAMP_AMPLIFICATION_ * (SHUNT_5V_MOHM_ / 1'000.f)); // mA
    static constexpr float CURRENT_SERVO_CONVERSION_FACTOR_ =
        ADC_VREF_MV_ / ((1 << CURRENT_ADC_RES_) - 1.f) / (OPAMP_AMPLIFICATION_ * (SHUNT_SERVO_MOHM_ / 1'000.f)); // mA
    static constexpr uint16_t CURRENT_SERVO_ADC_OFFSET_ { 60 };

    uint8_t last_adc_res_;
    std::mutex adc_mutex_;
    uint16_t line_[2];
    uint16_t border_[2];
    float bat_voltage_; // V
    float current_5v_; // mA
    float current_servo_; // mA

    /**
     * @brief Read all the current ADC values
     */
    void update();

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @return ADC value in 10 bit resolution
     */
    uint16_t analog_read(const uint8_t pin) {
        return analog_read(pin, 10, 1);
    }

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @param[in] resolution: Resolution in bit for ADC
     * @return ADC value in selected resolution
     */
    uint16_t analog_read(const uint8_t pin, const uint8_t resolution) {
        return analog_read(pin, resolution, 1);
    }

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @param[in] resolution: Resolution in bit for ADC
     * @param[in] avg_num: Number of reads to build an average
     * @return ADC value in selected resolution
     */
    uint16_t analog_read(const uint8_t pin, const uint8_t resolution, const uint8_t avg_num);

public:
    /**
     * @brief Construct a new AnalogSensors object
     */
    FLASHMEM AnalogSensors();

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

    auto get_bat_voltage() const {
        return bat_voltage_;
    }

    auto get_5v_current() const {
        return static_cast<uint16_t>(current_5v_);
    }

    auto get_servo_current() const {
        return static_cast<uint16_t>(current_servo_);
    }
};

} // namespace ctbot

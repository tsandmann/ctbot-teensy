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
 * @file    digital_sensors.h
 * @brief   Digital sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "ctbot_config.h"
#include "remote_control.h"

#include "driver/ena_i2c.h"
#include "driver/encoder.h"
#include "driver/leds_i2c.h"
#include "driver/rc5_int.h"

#include <array>
#include <cstdint>


namespace ctbot {
class CtBot;
class EnaI2c;
class VL53L0X;
class VL6180X;
class MPU6050_Wrapper;

/**
 * @brief Abstraction layer for (simple) digital sensors
 *
 * @startuml{DigitalSensors.png}
 *  !include digital_sensors.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class DigitalSensors {
protected:
    static constexpr bool DEBUG_ { false };

    static constexpr auto ENA_MASK_INIT = EnaI2cTypes::NONE; /*EnaI2cTypes::DISTANCE_L | EnaI2cTypes::DISTANCE_R | EnaI2cTypes::TRANSPORT; */
    static constexpr auto ENA_MASK = EnaI2cTypes::NONE;
    static constexpr auto ENA_MASK_PWM = LedTypesEna::NONE;
    static constexpr auto ENA_MASK_PWM_INIT = LedTypesEna::ENC_L | LedTypesEna::ENC_R;

    CtBot& ctbot_;
    EnaI2c& ena_;

    uint8_t transport_;

    Encoder enc_l_;
    Encoder enc_r_;
    Rc5 rc5_;
    RemoteControl remote_control_;
    std::array<uint16_t, 2> distance_;
    uint32_t dist_last_update_;
    uint32_t trans_last_update_;
    uint32_t mpu_last_update_;
    VL53L0X* p_dist_l;
    VL53L0X* p_dist_r;
    VL6180X* p_trans_;
    MPU6050_Wrapper* p_mpu_6050_;

    /**
     * @brief Construct a new DigitalSensors object
     */
    FLASHMEM DigitalSensors(CtBot& ctbot);

    /**
     * @brief Read all the current pin values
     */
    void update();

public:
    static uint32_t enc_data_l_[], enc_data_r_[]; /**< Raw input data buffers for both wheel encoders */
    static volatile uint8_t enc_l_idx_, enc_r_idx_; /**< Current indices in input data buffers, pointing to each latest entry */

    FLASHMEM ~DigitalSensors();

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
     * @return The last value of transport pocket occupancy
     */
    auto get_transport() const {
        return transport_ < 40U;
    }

    /**
     * @return The last sensor range of transport pocket
     */
    auto get_transport_mm() const {
        return transport_;
    }

    /**
     * @return Reference to the left wheel encoder driver
     */
    auto& get_enc_l() {
        return enc_l_;
    }

    /**
     * @return Reference to the right wheel encoder driver
     */
    auto& get_enc_r() {
        return enc_r_;
    }

    auto get_mpu6050() {
        return p_mpu_6050_;
    }

    /**
     * @return Reference to the IR receiver driver
     */
    auto& get_rc5() {
        return rc5_;
    }

    /**
     * @return Reference to the RC5 remote control driver
     */
    auto& get_rc() {
        return remote_control_;
    }
};

} // namespace ctbot

/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    mpu_6050.h
 * @brief   MPU 6050 sensor driver wrapper
 * @author  Timo Sandmann
 * @date    04.04.2021
 */

#pragma once

#include "helper_3dmath.h"

#include <cstdint>
#include <tuple>


class MPU6050;

namespace ctbot {
class MPU6050_Wrapper {
    static constexpr bool DEBUG_ { true };

    static constexpr std::tuple<int16_t, int16_t, int16_t> INITIAL_ACC_OFFSET_ { -32, 2514, 1376 };
    static constexpr std::tuple<int16_t, int16_t, int16_t> INITIAL_GYRO_OFFSET_ { 19, 42, 18 };

    const uint8_t i2c_bus_;
    const uint32_t i2c_freq_;
    MPU6050* p_mpu_;

    bool calc_euler_;
    bool calc_ypr_;
    bool calc_gyro_;
    bool calc_real_acc_;
    bool calc_word_acc_;

    uint8_t buffer_[64];

    Quaternion last_q_;
    std::tuple<float, float, float> last_euler_;
    VectorFloat last_g_;
    VectorInt16 last_a_;
    std::tuple<float, float, float> last_ypr_;
    VectorInt16 last_gyro_;
    VectorInt16 last_a_real_;
    VectorInt16 last_a_world_;

public:
    MPU6050_Wrapper(const uint8_t i2c_bus, const uint32_t i2c_freq);

    ~MPU6050_Wrapper();

    bool init();

    bool update();

    void clear_fifo() const;

    void enable_euler(const bool value) {
        calc_euler_ = value;
    }

    void enable_ypr(const bool value) {
        calc_ypr_ = value;
    }

    void enable_gyro(const bool value) {
        calc_gyro_ = value;
    }

    void enable_real_acc(const bool value) {
        calc_real_acc_ = value;
    }

    void enable_word_acc(const bool value) {
        calc_word_acc_ = value;
    }

    const auto& get_quaternion() const {
        return last_q_;
    }

    const auto& get_euler() const {
        return last_euler_;
    }

    const auto& get_ypr() const {
        return last_ypr_;
    }

    const auto& get_gravity() const {
        return last_g_;
    }

    const auto& get_gyro() const {
        return last_gyro_;
    }

    const auto& get_real_acc() const {
        return last_a_real_;
    }

    const auto& get_world_acc() const {
        return last_a_world_;
    }
};
} // namespace ctbot

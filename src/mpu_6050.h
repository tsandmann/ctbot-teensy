/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
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
 * @brief   MPU 6050 sensor driver
 * @author  Timo Sandmann
 * @date    17.11.2019
 */

#pragma once

#include "i2c_wrapper.h"

#include <cstdint>


namespace ctbot {
class I2C_Wrapper;

class MPU6050 {
    static constexpr uint8_t SMPLRT_DIV_REG { 0x19 };
    static constexpr uint8_t CONFIG_REG { 0x1a };
    static constexpr uint8_t GYRO_CONFIG_REG { 0x1b };
    static constexpr uint8_t ACCEL_CONFIG_REG { 0x1c };
    static constexpr uint8_t ACCEL_XOUT_H_REG { 0x3b };
    static constexpr uint8_t TEMP_H_REG { 0x41 };
    static constexpr uint8_t GYRO_XOUT_H_REG { 0x43 };
    static constexpr uint8_t SIGNAL_PATH_RESET_REG { 0x68 };
    static constexpr uint8_t PWR_MGMT_1_REG { 0x6b };

    static constexpr uint16_t CALIBRATION_RUNS { 100 };

    const float acc_coef_, gyro_coef_;

    I2C_Wrapper i2c_;

    std::array<float, 3> acc_;
    std::array<float, 3> gyro_;
    std::array<float, 3> gyro_offsets_;

    std::array<float, 3> angle_gyro_;
    std::array<float, 2> angle_acc_;
    std::array<float, 2> angles_;

    float temperature_;

    uint32_t last_update_;


public:
    static constexpr uint8_t DEFAULT_I2C_ADDR { 0x68 };

    MPU6050(const uint8_t i2c_bus, const uint8_t i2c_addr, const uint32_t i2c_freq);

    MPU6050(const uint8_t i2c_bus, const uint8_t i2c_addr, const uint32_t i2c_freq, const float acc_coef, const float gyro_coef);

    bool begin();

    bool calc_gyro_offset(const bool debug_out);

    bool update_gyro();

    bool update_acc();

    bool update_temp();

    bool update_all();

    void set_gyro_offset(const float x, const float y, const float z);

    auto get_temperature() const {
        return temperature_;
    };

    auto get_gyro_offset_x() const {
        return gyro_offsets_[0];
    };

    auto get_gyro_offset_y() const {
        return gyro_offsets_[1];
    };

    auto get_gyro_offset_z() const {
        return gyro_offsets_[2];
    };

    auto get_angle_acc_x() const {
        return angle_acc_[0];
    };

    auto get_angle_acc_y() const {
        return angle_acc_[1];
    };

    auto get_angle_gyro_x() const {
        return angle_gyro_[0];
    };

    auto get_angle_gyro_y() const {
        return angle_gyro_[1];
    };

    auto get_angle_gyro_z() const {
        return angle_gyro_[2];
    };

    auto get_angle_x() const {
        return angles_[0];
    };

    auto get_angle_y() const {
        return angles_[1];
    };
};
} // namespace ctbot

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
 * @file    mpu_6050.cpp
 * @brief   MPU 6050 sensor driver
 * @author  Timo Sandmann
 * @date    17.11.2019
 */

#include "mpu_6050.h"
#include "ctbot.h"

#include "pprintpp.hpp"

#include <chrono>
#include <thread>
#include <cmath>


namespace ctbot {
MPU6050::MPU6050(const uint8_t i2c_bus, const uint8_t i2c_addr, const uint32_t i2c_freq) : MPU6050 { i2c_bus, i2c_addr, i2c_freq, 0.02f, 0.98f } {}

MPU6050::MPU6050(const uint8_t i2c_bus, const uint8_t i2c_addr, const uint32_t i2c_freq, const float acc_coef, const float gyro_coef)
    : acc_coef_ { acc_coef }, gyro_coef_ { gyro_coef }, i2c_ { i2c_bus, i2c_addr, i2c_freq }, angle_gyro_ { 0.f, 0.f, 0.f },
      angle_acc_ { 0.f, 0.f }, angles_ { 0.f, 0.f }, temperature_ {}, last_update_ {} {}

bool MPU6050::begin() {
    if (!i2c_.init()) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): i2c_.init() failed.\r\n"), true);
        return false;
    }

    if (i2c_.write_reg8(PWR_MGMT_1_REG, 1 << 7)) { // reset sensor
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(PWR_MGMT_1_REG, 0x80) failed.\r\n"), true);
        return false;
    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);
    if (i2c_.write_reg8(SIGNAL_PATH_RESET_REG, 7)) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(SIGNAL_PATH_RESET_REG, 7) failed.\r\n"), true);
        return false;
    }
    std::this_thread::sleep_for(100ms);
    if (i2c_.write_reg8(PWR_MGMT_1_REG, 1)) { // clk source
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(PWR_MGMT_1_REG, 1) failed.\r\n"), true);
        return false;
    }
    std::this_thread::sleep_for(100ms);

    if (i2c_.write_reg8(SMPLRT_DIV_REG, 0)) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(SMPLRT_DIV_REG, 0) failed.\r\n"), true);
        return false;
    }
    if (i2c_.write_reg8(CONFIG_REG, 0)) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(CONFIG_REG, 0) failed.\r\n"), true);
        return false;
    }
    if (i2c_.write_reg8(GYRO_CONFIG_REG, 8)) { // FIXME: check value
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(GYRO_CONFIG_REG, 8) failed.\r\n"), true);
        return false;
    }
    if (i2c_.write_reg8(ACCEL_CONFIG_REG, 0)) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::begin(): write(ACCEL_CONFIG_REG, 0) failed.\r\n"), true);
        return false;
    }

    using namespace std::chrono;
    last_update_ = static_cast<uint32_t>(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());

    if (!update_gyro()) {
        return false;
    }

    return true;
}

bool MPU6050::calc_gyro_offset(const bool debug_out) {
    if (debug_out) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::calc_gyro_offset(): Calculating gyro offsets... DO NOT MOVE SENSOR\r\n"), true);
    }

    float x {}, y {}, z {};
    for (uint16_t i { 0 }; i < CALIBRATION_RUNS; ++i) {
        if (debug_out && i % (CALIBRATION_RUNS / 10) == 0) {
            CtBot::get_instance().get_comm()->debug_print('.', true);
        }
        std::array<uint8_t, 6> buf;
        if (i2c_.read_bytes(GYRO_XOUT_H_REG, buf.data(), buf.size())) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("\r\nMPU6050::calc_gyro_offset(): i2c error\r\n"), true);
            return false;
        }

        const int16_t rx { static_cast<int16_t>(buf[0] << 8 | buf[1]) };
        const int16_t ry { static_cast<int16_t>(buf[2] << 8 | buf[3]) };
        const int16_t rz { static_cast<int16_t>(buf[4] << 8 | buf[5]) };

        x += static_cast<float>(rx) / 65.5f;
        y += static_cast<float>(ry) / 65.5f;
        z += static_cast<float>(rz) / 65.5f;

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(5ms);
    }

    gyro_offsets_[0] = x / static_cast<float>(CALIBRATION_RUNS);
    gyro_offsets_[1] = y / static_cast<float>(CALIBRATION_RUNS);
    gyro_offsets_[2] = z / static_cast<float>(CALIBRATION_RUNS);

    if (debug_out) {
        CtBot::get_instance().get_comm()->debug_printf<true>(
            PP_ARGS("\r\nMPU6050::calc_gyro_offset(): Calibration done: X={}, Y={}, Z={}\r\n", gyro_offsets_[0], gyro_offsets_[1], gyro_offsets_[2]));
    }

    return true;
}

bool MPU6050::update_gyro() {
    std::array<uint8_t, 6> buf;
    if (i2c_.read_bytes(GYRO_XOUT_H_REG, buf.data(), buf.size())) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::update_gyro(): i2c error\r\n"), true);
        return false;
    }
    using namespace std::chrono;
    const auto now { static_cast<uint32_t>(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count()) };

    const int16_t raw_x { static_cast<int16_t>(buf[0] << 8 | buf[1]) };
    const int16_t raw_y { static_cast<int16_t>(buf[2] << 8 | buf[3]) };
    const int16_t raw_z { static_cast<int16_t>(buf[4] << 8 | buf[5]) };

    gyro_[0] = static_cast<float>(raw_x) / 65.5f;
    gyro_[1] = static_cast<float>(raw_y) / 65.5f;
    gyro_[2] = static_cast<float>(raw_z) / 65.5f;

    gyro_[0] -= gyro_offsets_[0];
    gyro_[1] -= gyro_offsets_[1];
    gyro_[2] -= gyro_offsets_[2];

    const auto interval { static_cast<float>(now - last_update_) / 1'000'000.f };
    last_update_ = now;

    const auto gyro_xi { gyro_[0] * interval };
    const auto gyro_yi { gyro_[1] * interval };
    const auto gyro_zi { gyro_[2] * interval };
    angle_gyro_[0] += gyro_xi;
    angle_gyro_[1] += gyro_yi;
    // angle_gyro_[2] += gyro_zi;
    angle_gyro_[2] = std::fmod(angle_gyro_[2] + gyro_zi, 360.f);

    angles_[0] = (gyro_coef_ * (angles_[0] + gyro_xi)) + (acc_coef_ * angle_acc_[0]);
    angles_[1] = (gyro_coef_ * (angles_[1] + gyro_yi)) + (acc_coef_ * angle_acc_[1]);

    return true;
}

bool MPU6050::update_acc() {
    std::array<uint8_t, 6> buf;
    if (i2c_.read_bytes(ACCEL_XOUT_H_REG, buf.data(), buf.size())) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::update_acc(): i2c error\r\n"), true);
        return false;
    }
    const int16_t raw_x { static_cast<int16_t>(buf[0] << 8 | buf[1]) };
    const int16_t raw_y { static_cast<int16_t>(buf[2] << 8 | buf[3]) };
    const int16_t raw_z { static_cast<int16_t>(buf[4] << 8 | buf[5]) };

    acc_[0] = static_cast<float>(raw_x) / 16384.f;
    acc_[1] = static_cast<float>(raw_y) / 16384.f;
    acc_[2] = static_cast<float>(raw_z) / 16384.f;

    angle_acc_[0] = std::atan2(acc_[1], acc_[2] + std::abs(acc_[0])) * (360.f / 2.f / M_PI);
    angle_acc_[1] = std::atan2(acc_[0], acc_[2] + std::abs(acc_[1])) * (360.f / -2.f / M_PI);

    return true;
}

bool MPU6050::update_temp() {
    std::array<uint8_t, 2> buf;
    if (i2c_.read_bytes(TEMP_H_REG, buf.data(), buf.size())) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050::update_temp(): i2c error\r\n"), true);
        return false;
    }

    const int16_t raw { static_cast<int16_t>(buf[0] << 8 | buf[1]) };
    temperature_ = (raw + 12412.f) / 340.f;

    return true;
}

bool MPU6050::update_all() {
    return update_acc() && update_gyro() && update_temp();
}

void MPU6050::set_gyro_offset(const float x, const float y, const float z) {
    gyro_offsets_[0] = x;
    gyro_offsets_[1] = y;
    gyro_offsets_[2] = z;
}

} // namespace ctbot

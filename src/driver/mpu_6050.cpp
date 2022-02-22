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
 * @file    mpu_6050.cpp
 * @brief   MPU 6050 sensor driver wrapper
 * @author  Timo Sandmann
 * @date    04.04.2021
 */

#include "ctbot.h"
#include "mpu_6050.h"

#include "MPU6050.h"
#include "arduino_freertos.h"


namespace ctbot {
MPU6050_Wrapper::MPU6050_Wrapper(const uint8_t i2c_bus, const uint32_t i2c_freq)
    : i2c_bus_ { i2c_bus }, i2c_freq_ { i2c_freq }, p_mpu_ {}, calc_euler_ {}, calc_ypr_ {}, calc_gyro_ {}, calc_real_acc_ {}, calc_word_acc_ {} {}

MPU6050_Wrapper::~MPU6050_Wrapper() {
    delete p_mpu_;
}

bool MPU6050_Wrapper::init() {
    if (!I2Cdev::init(i2c_bus_, i2c_freq_)) {
        return false;
    }

    p_mpu_ = new MPU6050;

    if (!p_mpu_) {
        return false;
    }

    p_mpu_->initialize();

    if (!p_mpu_->testConnection()) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050_Wrapper::init(): connection failure\r\n"), true);
        }
        return false;
    }

    const auto res { p_mpu_->dmpInitialize() };
    if (res) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("MPU6050_Wrapper::init(): DMP init failed: %u\r\n"), res);
        }
        return false;
    }

    p_mpu_->setXAccelOffset(std::get<0>(INITIAL_ACC_OFFSET_));
    p_mpu_->setYAccelOffset(std::get<1>(INITIAL_ACC_OFFSET_));
    p_mpu_->setZAccelOffset(std::get<2>(INITIAL_ACC_OFFSET_));
    p_mpu_->setXGyroOffset(std::get<0>(INITIAL_GYRO_OFFSET_));
    p_mpu_->setYGyroOffset(std::get<1>(INITIAL_GYRO_OFFSET_));
    p_mpu_->setZGyroOffset(std::get<2>(INITIAL_GYRO_OFFSET_));

    p_mpu_->CalibrateAccel(20);
    p_mpu_->CalibrateGyro(20);
    if (DEBUG_) {
        p_mpu_->PrintActiveOffsets();
    }

    p_mpu_->setDMPEnabled(true);
    if (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("DMP ready!\r\n"), true);
    }

    return true;
}

bool MPU6050_Wrapper::update() {
    const auto res { p_mpu_->dmpGetCurrentFIFOPacket(buffer_) };
    if (DEBUG_ && res == 0) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("MPU6050_Wrapper::update(): FIFO empty\r\n"), true);
        return false;
    }

    p_mpu_->dmpGetQuaternion(&last_q_, buffer_);

    if (calc_gyro_) {
        p_mpu_->dmpGetGyro(&last_gyro_, buffer_);
    }

    float tmp[3];
    if (calc_euler_) {
        p_mpu_->dmpGetEuler(tmp, &last_q_);
        std::get<0>(last_euler_) = tmp[0] * (180.f / M_PI);
        std::get<1>(last_euler_) = tmp[1] * (180.f / M_PI);
        std::get<2>(last_euler_) = tmp[2] * (180.f / M_PI);
    }

    if (calc_ypr_ | calc_real_acc_ | calc_word_acc_) {
        p_mpu_->dmpGetGravity(&last_g_, &last_q_);
    }

    if (calc_ypr_) {
        p_mpu_->dmpGetYawPitchRoll(tmp, &last_q_, &last_g_);
        std::get<0>(last_ypr_) = tmp[0] * (180.f / M_PI);
        std::get<1>(last_ypr_) = tmp[1] * (180.f / M_PI);
        std::get<2>(last_ypr_) = tmp[2] * (180.f / M_PI);
    }

    if (calc_real_acc_ | calc_word_acc_) {
        p_mpu_->dmpGetAccel(&last_a_, buffer_);
        p_mpu_->dmpGetLinearAccel(&last_a_real_, &last_a_, &last_g_);
    }

    if (calc_word_acc_) {
        p_mpu_->dmpGetLinearAccelInWorld(&last_a_world_, &last_a_real_, &last_q_);
    }

    return true;
}

void MPU6050_Wrapper::clear_fifo() const {
    p_mpu_->resetFIFO();
}

} // namespace ctbot

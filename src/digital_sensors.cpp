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
 * @file    digital_sensors.cpp
 * @brief   Digital sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "digital_sensors.h"
#include "ctbot.h"
#include "scheduler.h"
#include "vl53l0x.h"
#include "vl6180x.h"
#include "mpu_6050.h"

#include "arduino_fixed.h"


namespace ctbot {

std::remove_all_extents<decltype(DigitalSensors::enc_data_l_)>::type DigitalSensors::enc_data_l_[Encoder::DATA_ARRAY_SIZE],
    DigitalSensors::enc_data_r_[Encoder::DATA_ARRAY_SIZE];
decltype(DigitalSensors::enc_l_idx_) DigitalSensors::enc_l_idx_, DigitalSensors::enc_r_idx_;

DigitalSensors::DigitalSensors(CtBot& ctbot)
    : ctbot_ { ctbot }, ena_ { *ctbot_.get_ena() }, transport_ {}, enc_l_ { enc_data_l_, &enc_l_idx_, CtBotConfig::ENC_L_PIN },
      enc_r_ { enc_data_r_, &enc_r_idx_, CtBotConfig::ENC_R_PIN }, rc5_ { CtBotConfig::RC5_PIN }, remote_control_ { rc5_, CtBotConfig::RC5_ADDR },
      distance_ { 0, 0 }, dist_last_update_ {}, trans_last_update_ {}, p_dist_l {}, p_dist_r {}, p_trans_ {}, p_mpu_6050_ {} {
    ena_.off(EnaI2cTypes::DISTANCE_L | EnaI2cTypes::DISTANCE_R | EnaI2cTypes::TRANSPORT | EnaI2cTypes::EXTENSION_1); // shutdown i2c sensors
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);

    {
        ena_.on(EnaI2cTypes::DISTANCE_L);
        std::this_thread::sleep_for(10ms);
        p_dist_l = new VL53L0X { CtBotConfig::VL53L0X_I2C_BUS, CtBotConfig::I2C0_FREQ };
        configASSERT(p_dist_l);
        if (p_dist_l->init()) {
            if (!p_dist_l->set_address(CtBotConfig::VL53L0X_L_I2C_ADDR)) {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_dist_l->set_address() failed.\r\n", true);
                ena_.off(EnaI2cTypes::DISTANCE_L);
                delete p_dist_l;
                p_dist_l = nullptr;
            } else if (!p_dist_l->start_continuous(0)) {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_dist_l->start_continuous(0) failed.\r\n", true);
            }
        } else {
            ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_dist_l->init() failed.\r\n", true);
            ena_.off(EnaI2cTypes::DISTANCE_L);
            delete p_dist_l;
            p_dist_l = nullptr;
            // FIXME: error handling?
        }
    }

    {
        ena_.on(EnaI2cTypes::DISTANCE_R);
        std::this_thread::sleep_for(10ms);
        p_dist_r = new VL53L0X { CtBotConfig::VL53L0X_I2C_BUS, CtBotConfig::I2C0_FREQ };
        configASSERT(p_dist_r);
        if (p_dist_r->init()) {
            if (!p_dist_r->set_address(CtBotConfig::VL53L0X_R_I2C_ADDR)) {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_dist_r->set_address() failed.\r\n", true);
                ena_.off(EnaI2cTypes::DISTANCE_R);
                delete p_dist_r;
                p_dist_r = nullptr;
            } else if (!p_dist_r->start_continuous(0)) {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_dist_r->start_continuous(0) failed.\r\n", true);
            }
        } else {
            ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_dist_r->init() failed.\r\n", true);
            ena_.off(EnaI2cTypes::DISTANCE_R);
            delete p_dist_r;
            p_dist_r = nullptr;
            // FIXME: error handling?
        }
    }


    {
        ena_.on(EnaI2cTypes::TRANSPORT);
        std::this_thread::sleep_for(10ms);
        p_trans_ = new VL6180X { CtBotConfig::VL6180X_I2C_BUS, CtBotConfig::I2C0_FREQ };
        configASSERT(p_trans_);
        if (p_trans_->init()) {
            if (!p_trans_->set_address(CtBotConfig::VL6180X_I2C_ADDR)) {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_trans_->set_address() failed.\r\n", true);
                ena_.off(EnaI2cTypes::TRANSPORT);
                delete p_trans_;
                p_trans_ = nullptr;
            }
            if (p_trans_->configure_defaults()) {
                if (!p_trans_->start_continuous(100)) {
                    ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_trans_->start_continuous() failed.\r\n", true);
                }
            } else {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_trans_->configure_defaults() failed.\r\n", true);
            }
        } else {
            ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): p_trans_->init() failed.\r\n", true);
            ena_.off(EnaI2cTypes::TRANSPORT);
            delete p_trans_;
            p_trans_ = nullptr;
            // FIXME: error handling?
        }
    }

    if (CtBotConfig::MPU6050_AVAILABLE) {
        ena_.on(EnaI2cTypes::EXTENSION_1);
        std::this_thread::sleep_for(200ms);
        p_mpu_6050_ = new MPU6050 { CtBotConfig::MPU6050_I2C_BUS, MPU6050::DEFAULT_I2C_ADDR,
            CtBotConfig::MPU6050_I2C_BUS == 0 ?
                CtBotConfig::I2C0_FREQ :
                (CtBotConfig::MPU6050_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ :
                                                     (CtBotConfig::MPU6050_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ)) };
        configASSERT(p_mpu_6050_);
        if (!p_mpu_6050_->begin()) {
            if (DEBUG_) {
                ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): MPU6050 init failed.\r\n", true);
            }
            delete p_mpu_6050_;
            p_mpu_6050_ = nullptr;
        }
        if (p_mpu_6050_) {
            if (!p_mpu_6050_->calc_gyro_offset(DEBUG_)) {
                if (DEBUG_) {
                    ctbot_.get_comm()->debug_print("DigitalSensors::DigitalSensors(): MPU6050 calibration failed.\r\n", true);
                }
                delete p_mpu_6050_;
                p_mpu_6050_ = nullptr;
            }
        }
    }
}

DigitalSensors::~DigitalSensors() {
    delete p_mpu_6050_;
    delete p_dist_r;
    delete p_dist_l;
}

void DigitalSensors::update() {
    const auto now { Timer::get_ms() };

    enc_l_.update();
    enc_r_.update();
    rc5_.update();
    remote_control_.update();

    if (now - dist_last_update_ > 33) {
        dist_last_update_ = now;
        if (p_dist_l) {
            p_dist_l->get_dist_range(distance_[0]);
        }
        if (p_dist_r) {
            p_dist_r->get_dist_range(distance_[1]);
        }
    }

    if (now - trans_last_update_ > 200) {
        trans_last_update_ = now;
        if (p_trans_) {
            p_trans_->get_dist_range(transport_);
        }
    }

    if (CtBotConfig::MPU6050_AVAILABLE && p_mpu_6050_) {
        if (!p_mpu_6050_->update_gyro() && DEBUG_) {
            ctbot_.get_comm()->debug_print("DigitalSensors::update(): i2c error 4\r\n", true);
        }
    }
}

} // namespace ctbot

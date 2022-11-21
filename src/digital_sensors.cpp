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
 * @file    digital_sensors.cpp
 * @brief   Digital sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "digital_sensors.h"

#include "ctbot.h"
#include "scheduler.h"
#include "i2c_service.h"

#include "driver/mpu_6050.h"
#include "driver/vl53l0x.h"
#include "driver/vl6180x.h"

#include "arduino_freertos.h"


namespace ctbot {

std::remove_all_extents<decltype(DigitalSensors::enc_data_l_)>::type DigitalSensors::enc_data_l_[Encoder::DATA_ARRAY_SIZE],
    DigitalSensors::enc_data_r_[Encoder::DATA_ARRAY_SIZE];
decltype(DigitalSensors::enc_l_idx_) DigitalSensors::enc_l_idx_, DigitalSensors::enc_r_idx_;

DigitalSensors::DigitalSensors(CtBot& ctbot, I2C_Service* p_i2c_svc)
    : ctbot_ { ctbot }, ena_ { *ctbot_.get_ena() }, transport_ {}, enc_l_ { enc_data_l_, &enc_l_idx_,
          CtBotConfig::EXTERNAL_SPEEDCTRL ? 255 : CtBotConfig::ENC_L_PIN },
      enc_r_ { enc_data_r_, &enc_r_idx_, CtBotConfig::EXTERNAL_SPEEDCTRL ? 255 : CtBotConfig::ENC_R_PIN }, rc5_ { CtBotConfig::RC5_PIN },
      remote_control_ { rc5_, CtBotConfig::RC5_ADDR }, distance_ { 0, 0 }, dist_last_update_ {}, trans_last_update_ {}, mpu_last_update_ {},
      p_i2c_svc_ { p_i2c_svc }, p_dist_l {}, p_dist_r {}, p_trans_ {}, p_mpu_6050_ {} {
    ena_.off(EnaI2cTypes::DISTANCE_L | EnaI2cTypes::DISTANCE_R | EnaI2cTypes::TRANSPORT); // shutdown i2c sensors

    if (!p_i2c_svc_) {
        ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): no I2C service given, skipping I2C sensor initialization.\r\n"), true);
        return;
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);

    {
        ena_.on(EnaI2cTypes::DISTANCE_L);
        std::this_thread::sleep_for(10ms);
        p_dist_l = new VL53L0X { *p_i2c_svc_ };
        configASSERT(p_dist_l);
        if (p_dist_l->init()) {
            if (!p_dist_l->set_address(CtBotConfig::VL53L0X_L_I2C_ADDR)) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_dist_l->set_address() failed.\r\n"), true);
                ena_.off(EnaI2cTypes::DISTANCE_L);
                delete p_dist_l;
                p_dist_l = nullptr;
            } else if (!p_dist_l->start_continuous(0)) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_dist_l->start_continuous(0) failed.\r\n"), true);
            }
        } else {
            ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_dist_l->init() failed.\r\n"), true);
            ena_.off(EnaI2cTypes::DISTANCE_L);
            delete p_dist_l;
            p_dist_l = nullptr;
            // TODO: error handling?
        }
    }

    {
        ena_.on(EnaI2cTypes::DISTANCE_R);
        std::this_thread::sleep_for(10ms);
        p_dist_r = new VL53L0X { *p_i2c_svc_ };
        configASSERT(p_dist_r);
        if (p_dist_r->init()) {
            if (!p_dist_r->set_address(CtBotConfig::VL53L0X_R_I2C_ADDR)) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_dist_r->set_address() failed.\r\n"), true);
                ena_.off(EnaI2cTypes::DISTANCE_R);
                delete p_dist_r;
                p_dist_r = nullptr;
            } else if (!p_dist_r->start_continuous(0)) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_dist_r->start_continuous(0) failed.\r\n"), true);
            }
        } else {
            ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_dist_r->init() failed.\r\n"), true);
            ena_.off(EnaI2cTypes::DISTANCE_R);
            delete p_dist_r;
            p_dist_r = nullptr;
            // TODO: error handling?
        }
    }


    {
        ena_.on(EnaI2cTypes::TRANSPORT);
        std::this_thread::sleep_for(10ms);
        p_trans_ = new VL6180X { *p_i2c_svc_ };
        configASSERT(p_trans_);
        if (p_trans_->init()) {
            if (!p_trans_->set_address(CtBotConfig::VL6180X_I2C_ADDR)) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_trans_->set_address() failed.\r\n"), true);
                ena_.off(EnaI2cTypes::TRANSPORT);
                delete p_trans_;
                p_trans_ = nullptr;
            }
            if (p_trans_->configure_defaults()) {
                if (!p_trans_->start_continuous(100)) {
                    ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_trans_->start_continuous() failed.\r\n"), true);
                }
            } else {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_trans_->configure_defaults() failed.\r\n"), true);
            }
        } else {
            ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): p_trans_->init() failed.\r\n"), true);
            ena_.off(EnaI2cTypes::TRANSPORT);
            delete p_trans_;
            p_trans_ = nullptr;
            // TODO: error handling?
        }
    }

    if (CtBotConfig::MPU6050_AVAILABLE) {
        p_mpu_6050_ = new MPU6050_Wrapper { CtBotConfig::IMU_I2C_BUS - 1,
            CtBotConfig::IMU_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ : (CtBotConfig::IMU_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ) };
        configASSERT(p_mpu_6050_);
        if (p_mpu_6050_->init()) {
            p_mpu_6050_->enable_ypr(true);
            p_mpu_6050_->enable_euler(true);
        } else {
            if (DEBUG_) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::DigitalSensors(): MPU6050 init failed.\r\n"), true);
            }
            delete p_mpu_6050_;
            p_mpu_6050_ = nullptr;
        }
    }
}

DigitalSensors::~DigitalSensors() {
    delete p_mpu_6050_;
    delete p_trans_;
    delete p_dist_r;
    delete p_dist_l;
}

void DigitalSensors::update() {
    const auto now { Timer::get_ms() };

    if (!CtBotConfig::EXTERNAL_SPEEDCTRL) {
        enc_l_.update();
        enc_r_.update();
    }
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
        if (now - mpu_last_update_ >= 12) {
            mpu_last_update_ = now;
            if (!p_mpu_6050_->update() && DEBUG_) {
                ctbot_.get_comm()->debug_print(PSTR("DigitalSensors::update(): p_mpu_6050_->update() failed\r\n"), true);
            }
            p_mpu_6050_->clear_fifo();
        }
    }
}

} // namespace ctbot

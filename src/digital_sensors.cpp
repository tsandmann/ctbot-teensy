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

#include "driver/vl53l0x.h"
#include "driver/vl6180x.h"
#include "driver/fxas21002c.h"
#include "driver/fxos8700.h"
#include "driver/mpu_6050.h"

#include "arduino_freertos.h"


namespace ctbot {

std::remove_all_extents<decltype(DigitalSensors::enc_data_l_)>::type DigitalSensors::enc_data_l_[Encoder::DATA_ARRAY_SIZE],
    DigitalSensors::enc_data_r_[Encoder::DATA_ARRAY_SIZE];
decltype(DigitalSensors::enc_l_idx_) DigitalSensors::enc_l_idx_, DigitalSensors::enc_r_idx_;

DigitalSensors::DigitalSensors(CtBot& ctbot, I2C_Service* p_i2c_1_svc, I2C_Service* p_i2c_2_svc)
    : ctbot_ { ctbot }, ena_ { *ctbot_.get_ena() }, transport_ {}, enc_l_ { enc_data_l_, &enc_l_idx_,
          CtBotConfig::EXTERNAL_SPEEDCTRL ? 255 : CtBotConfig::ENC_L_PIN },
      enc_r_ { enc_data_r_, &enc_r_idx_, CtBotConfig::EXTERNAL_SPEEDCTRL ? 255 : CtBotConfig::ENC_R_PIN }, rc5_ { CtBotConfig::RC5_PIN },
      remote_control_ { rc5_, CtBotConfig::RC5_ADDR }, distance_ { 0, 0 }, dist_last_update_ {}, trans_last_update_ {}, mpu_last_update_ {},
      p_i2c_1_svc_ { p_i2c_1_svc }, p_i2c_2_svc_ { p_i2c_2_svc }, p_dist_l {}, p_dist_r {}, p_trans_ {}, p_gyro_ {}, p_accel_ {}, p_mpu_6050_ {} {
    auto p_logger { ctbot_.get_logger() };
    configASSERT(p_logger);
    auto log_begin { [p_logger]() { p_logger->begin(PSTR("DigitalSensors::DigitalSensors(): ")); } };

    ena_.off(EnaI2cTypes::DISTANCE_L | EnaI2cTypes::DISTANCE_R | EnaI2cTypes::TRANSPORT | EnaI2cTypes::IMU_RESET); // shutdown i2c sensors

    if (!p_i2c_1_svc) {
        log_begin();
        p_logger->log(PSTR("no I2C service for bus 1 given, skipping I2C sensor initialization.\r\n"), true);
        return;
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);

    {
        ena_.on(EnaI2cTypes::DISTANCE_L);
        std::this_thread::sleep_for(10ms);
        p_dist_l = new VL53L0X { *p_i2c_1_svc };
        configASSERT(p_dist_l);
        if (p_dist_l->init()) {
            if (!p_dist_l->set_address(CtBotConfig::VL53L0X_L_I2C_ADDR)) {
                log_begin();
                p_logger->log(PSTR("p_dist_l->set_address() failed.\r\n"), true);
                ena_.off(EnaI2cTypes::DISTANCE_L);
                delete p_dist_l;
                p_dist_l = nullptr;
            } else if (!p_dist_l->start_continuous(0)) {
                log_begin();
                p_logger->log(PSTR("p_dist_l->start_continuous(0) failed.\r\n"), true);
            }
        } else {
            log_begin();
            p_logger->log(PSTR("p_dist_l->init() failed.\r\n"), true);
            ena_.off(EnaI2cTypes::DISTANCE_L);
            delete p_dist_l;
            p_dist_l = nullptr;
            // TODO: error handling?
        }
    }

    {
        ena_.on(EnaI2cTypes::DISTANCE_R);
        std::this_thread::sleep_for(10ms);
        p_dist_r = new VL53L0X { *p_i2c_1_svc };
        configASSERT(p_dist_r);
        if (p_dist_r->init()) {
            if (!p_dist_r->set_address(CtBotConfig::VL53L0X_R_I2C_ADDR)) {
                log_begin();
                p_logger->log(PSTR("p_dist_r->set_address() failed.\r\n"), true);
                ena_.off(EnaI2cTypes::DISTANCE_R);
                delete p_dist_r;
                p_dist_r = nullptr;
            } else if (!p_dist_r->start_continuous(0)) {
                log_begin();
                p_logger->log(PSTR("p_dist_r->start_continuous(0) failed.\r\n"), true);
            }
        } else {
            log_begin();
            p_logger->log(PSTR("p_dist_r->init() failed.\r\n"), true);
            ena_.off(EnaI2cTypes::DISTANCE_R);
            delete p_dist_r;
            p_dist_r = nullptr;
            // TODO: error handling?
        }
    }

    {
        ena_.on(EnaI2cTypes::TRANSPORT);
        std::this_thread::sleep_for(10ms);
        p_trans_ = new VL6180X { *p_i2c_1_svc };
        configASSERT(p_trans_);
        if (p_trans_->init()) {
            if (!p_trans_->set_address(CtBotConfig::VL6180X_I2C_ADDR)) {
                log_begin();
                p_logger->log(PSTR("p_trans_->set_address() failed.\r\n"), true);
                ena_.off(EnaI2cTypes::TRANSPORT);
                delete p_trans_;
                p_trans_ = nullptr;
            }
            if (p_trans_->configure_defaults()) {
                if (!p_trans_->start_continuous(100)) {
                    log_begin();
                    p_logger->log(PSTR("p_trans_->start_continuous() failed.\r\n"), true);
                }
            } else {
                log_begin();
                p_logger->log(PSTR("p_trans_->configure_defaults() failed.\r\n"), true);
            }
        } else {
            log_begin();
            p_logger->log(PSTR("p_trans_->init() failed.\r\n"), true);
            ena_.off(EnaI2cTypes::TRANSPORT);
            delete p_trans_;
            p_trans_ = nullptr;
            // TODO: error handling?
        }
    }

    if (CtBotConfig::FXA_FXO_AVAILABLE && p_i2c_2_svc_) {
        ena_.on(EnaI2cTypes::IMU_RESET);
        std::this_thread::sleep_for(10ms);
        if constexpr (CtBotConfig::GYRO_AVAILABLE) {
            p_gyro_ = new FXAS21002C { p_i2c_2_svc_, 0x21 };
            configASSERT(p_gyro_);
            if (p_gyro_->begin()) {
                p_gyro_->set_range(FXAS21002C::Range::DPS_250);
                p_gyro_->set_odr(100.f);
                log_begin();
                p_logger->log(PSTR("Starting gyro calibration...\r\n"), true);
                p_gyro_->calibrate();
                log_begin();
                p_logger->log(PSTR("Gyro calibration done.\r\n"), true);
            } else {
                if (DEBUG_) {
                    log_begin();
                    p_logger->log(PSTR("FXAS21002C begin() failed.\r\n"), true);
                }
                delete p_gyro_;
                p_gyro_ = nullptr;
                // TODO: error handling?
            }
        }

        if constexpr (CtBotConfig::ACCEL_AVAILABLE) {
            p_accel_ = new FXOS8700 { p_i2c_2_svc_, 0x1f };
            configASSERT(p_accel_);
            if (p_accel_->begin()) {
                std::this_thread::sleep_for(200ms);
                p_accel_->set_sensor_mode(FXOS8700::sensor_mode_t::ACCEL_ONLY_MODE);
                p_accel_->set_output_data_rate(FXOS8700::odr_t::ODR_100HZ);
                p_accel_->set_accel_range(FXOS8700::accel_range_t::ACCEL_RANGE_2G);
                log_begin();
                p_logger->log(PSTR("Starting accelerometer calibration...\r\n"), true);
                p_accel_->calibrate();
                log_begin();
                p_logger->log(PSTR("Accelerometer calibration done.\r\n"), true);

            } else {
                if (DEBUG_) {
                    log_begin();
                    p_logger->log(PSTR("FXOS8700 begin() failed.\r\n"), true);
                }
                delete p_accel_;
                p_accel_ = nullptr;
                // TODO: error handling?
            }
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
                log_begin();
                p_logger->log(PSTR("MPU6050 init() failed.\r\n"), true);
            }
            delete p_mpu_6050_;
            p_mpu_6050_ = nullptr;
        }
    }
}

DigitalSensors::~DigitalSensors() {
    delete p_mpu_6050_;
    delete p_accel_;
    delete p_gyro_;
    delete p_trans_;
    delete p_dist_r;
    delete p_dist_l;
}

void DigitalSensors::update() {
    const auto now { Timer::get_ms() };

    auto p_logger { ctbot_.get_logger() };
    configASSERT(p_logger);
    auto log_begin { [p_logger]() { p_logger->begin(PSTR("DigitalSensors::update()")); } };

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

    if (CtBotConfig::FXA_FXO_AVAILABLE && CtBotConfig::GYRO_AVAILABLE && p_gyro_) {
        if (!p_gyro_->update()) {
            log_begin();
            p_logger->log(PSTR("p_gyro_->update() failed\r\n"), true);
        }
    }

    if (CtBotConfig::FXA_FXO_AVAILABLE && CtBotConfig::ACCEL_AVAILABLE && p_accel_) {
        if (!p_accel_->update()) {
            log_begin();
            p_logger->log(PSTR("p_accel_->update() failed\r\n"), true);
        }
    }

    if (CtBotConfig::MPU6050_AVAILABLE && p_mpu_6050_) {
        if (now - mpu_last_update_ >= 12) {
            mpu_last_update_ = now;
            if (!p_mpu_6050_->update() && DEBUG_) {
                log_begin();
                p_logger->log(PSTR("p_mpu_6050_->update() failed\r\n"), true);
            }
            p_mpu_6050_->clear_fifo();
        }
    }
}

} // namespace ctbot

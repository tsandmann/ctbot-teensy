/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2022 Timo Sandmann
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
 * @file    fxos8700.cpp
 * @brief   FXOS8700 sensor driver
 * @author  Timo Sandmann
 * @date    20.11.2022
 */

#include "fxos8700.h"
#include "timer.h"

#include <array>


namespace ctbot {
FXOS8700::FXOS8700(I2C_Service* p_i2c_svc, uint8_t dev_addr)
    : dev_addr_ { dev_addr }, p_i2c_svc_ { p_i2c_svc }, mode_ { sensor_mode_t::HYBRID_MODE }, acc_range_factor_ {}, range_ { accel_range_t::ACCEL_RANGE_2G },
      rate_ { odr_t::ODR_100HZ }, ratio_ { mag_osr_t::MAG_OSR_7 }, accel_raw_ {}, accel_cal_ {}, last_accel_ {}, mag_raw_ {}, last_mag_ {}, last_update_us_ {} {
}

bool FXOS8700::begin() {
    uint8_t id {};
    if (p_i2c_svc_->read_reg(dev_addr_, static_cast<uint8_t>(REGISTER_WHO_AM_I_), id) || id != DEVICE_ID_) {
        if constexpr (DEBUG_) {
            arduino::Serial.printf(PSTR("FXOS8700::begin(): ID read failed, id=0x%x\r\n"), id);
        }
        return false;
    }

    /* Set the full scale range of the accelerometer */
    set_accel_range(accel_range_t::ACCEL_RANGE_2G);

    /* Low Noise & High accelerometer OSR resolution */
    if (!standby(true)) {
        return false;
    }
    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), 0b100, 0b100)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::begin(): set_bits 1 failed"));
        }
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG2_), 0b11, 0b10)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::begin(): set_bits 2 failed"));
        }
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_MCTRL_REG1_), 0b10000000, 0b10000000)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::begin(): set_bits 3 failed"));
        }
        return false;
    }


    if (!standby(false)) {
        return false;
    }

    /* Set in hybrid mode, jumps to reg 0x33 after reading 0x06 */
    set_sensor_mode(sensor_mode_t::HYBRID_MODE);
    /* Set the output data rate to 100Hz, default */
    set_output_data_rate(odr_t::ODR_100HZ);

    /* Highest Over Sampling Ratio = 7 > (Over Sampling Rate = 16 @ 100Hz ODR) */
    set_mag_oversampling_ratio(mag_osr_t::MAG_OSR_7);

    std::apply([](auto&&... elem) { ((elem = 0), ...); }, accel_raw_);
    std::apply([](auto&&... elem) { ((elem = 0), ...); }, accel_cal_);
    std::apply([](auto&&... elem) { ((elem = 0.f), ...); }, last_accel_);
    std::apply([](auto&&... elem) { ((elem = 0), ...); }, mag_raw_);
    std::apply([](auto&&... elem) { ((elem = 0.f), ...); }, last_mag_);

    if constexpr (DEBUG_) {
        arduino::Serial.println(PSTR("FXOS8700::begin(): done."));
    }

    return true;
}

bool FXOS8700::calibrate() {
    static constexpr float APLHA { 0.01f };

    if (mode_ == sensor_mode_t::MAG_ONLY_MODE) {
        return false;
    }

    std::apply([](auto&&... elem) { ((elem = 0), ...); }, accel_cal_);

    if (!update()) {
        return false;
    }

    auto& [acc_raw_x, acc_raw_y, acc_raw_z] { accel_raw_ };
    float x { static_cast<float>(acc_raw_x) }, y { static_cast<float>(acc_raw_y) }, z { static_cast<float>(acc_raw_z) };

    for (size_t i {}; i < 200; ++i) {
        if (!update()) {
            return false;
        }

        x = (APLHA * acc_raw_x) + (1 - APLHA) * x;
        y = (APLHA * acc_raw_y) + (1 - APLHA) * y;
        z = (APLHA * acc_raw_z) + (1 - APLHA) * z;

        const uint32_t sleep { rate_ <= odr_t::ODR_50HZ ? 20u : 40u };
        ::vTaskDelay(pdMS_TO_TICKS(sleep));
    }

    auto& [cal_x, cal_y, cal_z] { accel_cal_ };
    cal_x = static_cast<int16_t>(x);
    cal_y = static_cast<int16_t>(y);
    cal_z = static_cast<int16_t>(z);

    if constexpr (DEBUG_) {
        arduino::Serial.printf(PSTR("FXOS8700::calibrate(): accel_cal_={%d, %d, %d}\r\n"), cal_x, cal_y, cal_z);
    }

    return true;
}

bool FXOS8700::update() {
    const auto now { Timer::get_us() };
    std::array<uint8_t, 13> buf;
    if (p_i2c_svc_->read_bytes(dev_addr_, static_cast<uint8_t>(REGISTER_STATUS_), buf.data(), buf.size())) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::getEvent(): read_bytes() failed."));
        }
        return false;
    }

    if (mode_ == sensor_mode_t::ACCEL_ONLY_MODE || mode_ == sensor_mode_t::HYBRID_MODE) {
        auto& [acc_raw_x, acc_raw_y, acc_raw_z] { accel_raw_ };
        acc_raw_x = (static_cast<int16_t>((buf[1] << 8) | buf[2]) >> 2) - std::get<0>(accel_cal_);
        acc_raw_y = (static_cast<int16_t>((buf[3] << 8) | buf[4]) >> 2) - std::get<1>(accel_cal_);
        acc_raw_z = (static_cast<int16_t>((buf[5] << 8) | buf[6]) >> 2) - std::get<2>(accel_cal_);

        const auto tmp_x { acc_raw_x * acc_range_factor_ };
        const auto tmp_y { acc_raw_y * acc_range_factor_ };
        const auto tmp_z { acc_raw_z * acc_range_factor_ };

        auto& [x, y, z] { last_accel_ };
        ::vTaskSuspendAll();
        x = tmp_x;
        y = tmp_y;
        z = tmp_z;
        last_update_us_ = now;
        ::xTaskResumeAll();
    }

    if (mode_ == sensor_mode_t::MAG_ONLY_MODE || mode_ == sensor_mode_t::HYBRID_MODE) {
        auto& [mag_raw_x, mag_raw_y, mag_raw_z] { mag_raw_ };
        mag_raw_x = static_cast<int16_t>((buf[7] << 8) | buf[8]);
        mag_raw_y = static_cast<int16_t>((buf[9] << 8) | buf[10]);
        mag_raw_z = static_cast<int16_t>((buf[11] << 8) | buf[12]);

        const auto tmp_x { mag_raw_x * MAG_UT_LSB_ };
        const auto tmp_y { mag_raw_y * MAG_UT_LSB_ };
        const auto tmp_z { mag_raw_z * MAG_UT_LSB_ };

        auto& [x, y, z] { last_mag_ };
        ::vTaskSuspendAll();
        x = tmp_x;
        y = tmp_y;
        z = tmp_z;
        last_update_us_ = now;
        ::xTaskResumeAll();
    }

    return true;
}

bool FXOS8700::standby(bool standby) const {
    const uint8_t mask { static_cast<uint8_t>(standby ? 0 : 0b1) };
    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), 0b1, mask)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::standby(): set_bits 1 failed"));
        }
        return false;
    }

    uint8_t tmp;
    do {
        if (p_i2c_svc_->read_reg(dev_addr_, static_cast<uint8_t>(REGISTER_SYSMOD_), tmp)) {
            if constexpr (DEBUG_) {
                arduino::Serial.println(PSTR("FXOS8700::standby(): read_reg 1 failed"));
            }
            return false;
        }
    } while (standby ? tmp & 0b11 : (tmp & 0b11) == 0); // FIXME: timeout

    return true;
}

bool FXOS8700::set_sensor_mode(sensor_mode_t mode) {
    if (!standby(true)) {
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_MCTRL_REG1_), 0b11, static_cast<uint8_t>(mode))) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::set_sensor_mode(): set_bits 1 failed"));
        }
        standby(false);
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_MCTRL_REG2_), 0b10000, mode == sensor_mode_t::HYBRID_MODE ? 0b10000 : 0b00000)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::set_sensor_mode(): set_bits 2 failed"));
        }
        standby(false);
        return false;
    }

    mode_ = mode;
    return standby(false);
}

bool FXOS8700::set_accel_range(accel_range_t range) {
    if (!standby(true)) {
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_XYZ_DATA_CFG_), 0b11, static_cast<uint8_t>(range))) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::set_accel_range(): set_bits 1 failed"));
        }
        standby(false);
        return false;
    }

    if (range == accel_range_t::ACCEL_RANGE_8G) {
        if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), 0b100, 0b00)) {
            if constexpr (DEBUG_) {
                arduino::Serial.println(PSTR("FXOS8700::set_accel_range(): set_bits 2 failed"));
            }
            standby(false);
            return false;
        }
    }

    switch (range) {
        case accel_range_t::ACCEL_RANGE_2G: acc_range_factor_ = ACCEL_MG_LSB_2G_ * ACCEL_GRAVITY_EARTH_; break;
        case accel_range_t::ACCEL_RANGE_4G: acc_range_factor_ = ACCEL_MG_LSB_4G_ * ACCEL_GRAVITY_EARTH_; break;
        case accel_range_t::ACCEL_RANGE_8G: acc_range_factor_ = ACCEL_MG_LSB_8G_ * ACCEL_GRAVITY_EARTH_; break;
    }
    range_ = range;

    return standby(false);
}

bool FXOS8700::set_output_data_rate(odr_t rate) {
    bool is_rate_in_mode {};
    uint8_t odr;

    if (mode_ == sensor_mode_t::HYBRID_MODE) {
        /* test if rate param belongs in the available hybrid ODR mode options */
        for (size_t i {}; i < 8; ++i) {
            if (rate == HYBRID_AVAILABLE_ODRS_[i]) {
                odr = ODR_DR_BITS_[i];
                is_rate_in_mode = true;
                break;
            }
        }
    } else {
        /* test if rate param belongs in the available accel/mag-only ODR mode options */
        for (size_t i {}; i < 8; ++i) {
            if (rate == ACCEL_MAG_ONLY_AVAILABLE_ODRS_[i]) {
                odr = ODR_DR_BITS_[i];
                is_rate_in_mode = true;
                break;
            }
        }
    }

    if (!is_rate_in_mode) {
        /* requested rate can't be set in current sensor mode */
        return false;
    }

    if (!standby(true)) {
        return false;
    }

    if (p_i2c_svc_->write_reg(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), odr)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::set_output_data_rate(): write_reg failed"));
        }
        standby(false);
        return false;
    }

    rate_ = rate;
    return standby(false);
}

bool FXOS8700::set_mag_oversampling_ratio(mag_osr_t ratio) {
    if (!standby(true)) {
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_MCTRL_REG1_), 0b11100, static_cast<uint8_t>(ratio) << 2)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXOS8700::set_mag_oversampling_ratio(): set_bits 1 failed"));
        }
        standby(false);
        return false;
    }

    ratio_ = ratio;
    return standby(false);
}

} // namespace ctbot

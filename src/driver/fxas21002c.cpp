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
 * @file    fxas21002c.cpp
 * @brief   FXAS21002C sensor driver
 * @author  Timo Sandmann
 * @date    20.11.2022
 */

#include "fxas21002c.h"
#include "timer.h"

#include <array>


namespace ctbot {
FXAS21002C::FXAS21002C(I2C_Service* p_i2c_svc, uint8_t dev_addr)
    : dev_addr_ { dev_addr }, p_i2c_svc_ { p_i2c_svc }, range_factor_ {}, odr_ {}, raw_ {}, cal_ {}, last_rate_ {}, last_update_us_ {} {}

bool FXAS21002C::begin() {
    uint8_t id {};
    if (p_i2c_svc_->read_reg(dev_addr_, static_cast<uint8_t>(REGISTER_WHO_AM_I_), id) || id != DEVICE_ID_) {
        if constexpr (DEBUG_) {
            arduino::Serial.printf(PSTR("FXAS21002C::begin(): ID read failed, id=0x%x\r\n"), id);
        }
        return false;
    }

    range_factor_ = static_cast<float>(Range::DPS_2000) / SENSITIVITY_FACTOR_;
    odr_ = 800.f;
    std::apply([](auto&&... elem) { ((elem = 0), ...); }, raw_);
    std::apply([](auto&&... elem) { ((elem = 0), ...); }, cal_);
    std::apply([](auto&&... elem) { ((elem = 0.f), ...); }, last_rate_);

    /* reset */
    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), 0b1111111, 0b1000000)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXAS21002C::begin(): set_bits() 1 failed."));
        }
        return false;
    }

    /* set active */
    if (!standby(false)) {
        return false;
    }

    if constexpr (DEBUG_) {
        arduino::Serial.println(PSTR("FXAS21002C::begin() done."));
    }

    return true;
}

bool FXAS21002C::calibrate() {
    static constexpr float APLHA { 0.01f };
    std::apply([](auto&&... elem) { ((elem = 0), ...); }, cal_);

    if (!update()) {
        return false;
    }

    auto& [raw_x, raw_y, raw_z] { raw_ };
    float x { static_cast<float>(raw_x) }, y { static_cast<float>(raw_y) }, z { static_cast<float>(raw_z) };
    for (size_t i {}; i < 200; ++i) {
        if (!update()) {
            return false;
        }

        x = (APLHA * raw_x) + (1 - APLHA) * x;
        y = (APLHA * raw_y) + (1 - APLHA) * y;
        z = (APLHA * raw_z) + (1 - APLHA) * z;

        const uint32_t sleep { static_cast<uint32_t>(.5f + 1'000.f / odr_) };
        ::vTaskDelay(pdMS_TO_TICKS(sleep));
    }

    auto& [cal_x, cal_y, cal_z] { cal_ };
    cal_x = static_cast<int16_t>(x);
    cal_y = static_cast<int16_t>(y);
    cal_z = static_cast<int16_t>(z);

    if constexpr (DEBUG_) {
        arduino::Serial.printf(PSTR("FXAS21002C::calibrate(): cal={%d, %d, %d}\r\n"), cal_x, cal_y, cal_z);
    }

    return true;
}

bool FXAS21002C::update() {
    const auto now { Timer::get_us() };
    std::array<uint8_t, 7> buf;
    if (p_i2c_svc_->read_bytes(dev_addr_, static_cast<uint8_t>(REGISTER_STATUS_), buf.data(), buf.size())) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXAS21002C::update(): read_bytes() failed."));
        }
        return false;
    }

    auto& [raw_x, raw_y, raw_z] { raw_ };
    raw_x = static_cast<int16_t>((buf[1] << 8) | buf[2]) - std::get<0>(cal_);
    raw_y = static_cast<int16_t>((buf[3] << 8) | buf[4]) - std::get<1>(cal_);
    raw_z = static_cast<int16_t>((buf[5] << 8) | buf[6]) - std::get<2>(cal_);

    const auto tmp_x { raw_x * range_factor_ };
    const auto tmp_y { raw_y * range_factor_ };
    const auto tmp_z { raw_z * range_factor_ };

    auto& [x, y, z] { last_rate_ };
    ::vTaskSuspendAll();
    x = tmp_x;
    y = tmp_y;
    z = tmp_z;
    last_update_us_ = now;
    ::xTaskResumeAll();

    return true;
}

bool FXAS21002C::standby(bool standby) const {
    const uint8_t mask { static_cast<uint8_t>(standby ? 0 : 0b11) };
    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), 0b11, mask)) {
        if constexpr (DEBUG_) {
            arduino::Serial.printf(PSTR("FXAS21002C::standby(%u): set_bits() failed.\r\n"), standby);
        }
        return false;
    }
    if (!standby) {
        const uint32_t sleep { static_cast<uint32_t>(60.5f + 1'000.f / odr_) };
        ::vTaskDelay(pdMS_TO_TICKS(sleep));
    }

    return true;
}

bool FXAS21002C::set_range(Range range) {
    if (!standby(true)) {
        return false;
    }

    uint8_t mask {};
    switch (range) {
        case Range::DPS_250: mask = 0b11; break;
        case Range::DPS_500: mask = 0b10; break;
        case Range::DPS_1000: mask = 0b01; break;
        case Range::DPS_2000: mask = 0b00; break;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG0_), 0b11, mask)) {
        if constexpr (DEBUG_) {
            arduino::Serial.println(PSTR("FXAS21002C::set_range(): set_bits() failed."));
        }
        standby(false);
        return false;
    }

    range_factor_ = static_cast<float>(range) / SENSITIVITY_FACTOR_;
    return standby(false);
}

bool FXAS21002C::set_odr(float odr) {
    uint8_t mask {};
    if (odr == 800.f) {
        mask = 0b00000;
    } else if (odr == 400.f) {
        mask = 0b00100;
    } else if (odr == 200.f) {
        mask = 0b01000;
    } else if (odr == 100.f) {
        mask = 0b01100;
    } else if (odr == 50.f) {
        mask = 0b10000;
    } else if (odr == 25.f) {
        mask = 0b10100;
    } else if (odr == 12.5f) {
        mask = 0b11000;
    } else {
        return false;
    }

    if (!standby(true)) {
        return false;
    }

    if (p_i2c_svc_->set_bits(dev_addr_, static_cast<uint8_t>(REGISTER_CTRL_REG1_), 0b11100, mask)) {
        if constexpr (DEBUG_) {
            Serial.println(PSTR("FXAS21002C::set_odr(): set_bits() failed."));
        }
        standby(false);
        return false;
    }

    odr_ = odr;

    return standby(false);
}

} // namespace ctbot

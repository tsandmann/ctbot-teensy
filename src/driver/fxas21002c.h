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
 * @file    fxas21002c.h
 * @brief   FXAS21002C sensor driver
 * @author  Timo Sandmann
 * @date    20.11.2022
 */

#pragma once

#include "i2c_service.h"

#include <cstdint>
#include <tuple>


namespace ctbot {
class FXAS21002C {
    static constexpr bool DEBUG_ { false };

protected:
    static constexpr uint8_t DEVICE_ID_ { 0xd7 };

    static constexpr uint8_t REGISTER_STATUS_ { 0 };
    static constexpr uint8_t REGISTER_OUT_X_MSB_ { 1 };
    static constexpr uint8_t REGISTER_OUT_X_LSB_ { 2 };
    static constexpr uint8_t REGISTER_OUT_Y_MSB_ { 3 };
    static constexpr uint8_t REGISTER_OUT_Y_LSB_ { 4 };
    static constexpr uint8_t REGISTER_OUT_Z_MSB_ { 5 };
    static constexpr uint8_t REGISTER_OUT_Z_LSB_ { 6 };
    static constexpr uint8_t REGISTER_WHO_AM_I_ { 0xc };
    static constexpr uint8_t REGISTER_CTRL_REG0_ { 0xd };
    static constexpr uint8_t REGISTER_CTRL_REG1_ { 0x13 };
    static constexpr uint8_t REGISTER_CTRL_REG2_ { 0x14 };

    static constexpr float SENSITIVITY_FACTOR_ { 32'000.f };

public:
    enum class Range : uint16_t { DPS_250 = 250, DPS_500 = 500, DPS_1000 = 1'000, DPS_2000 = 2'000 };

    FXAS21002C(I2C_Service* p_i2c_svc, uint8_t dev_addr = 0x21);

    ~FXAS21002C() = default;

    bool begin();

    bool calibrate();

    bool update();

    bool standby(bool standby) const;

    bool set_range(Range range);

    bool set_odr(float odr);

    int16_t get_range() const {
        return static_cast<int16_t>(range_factor_ * SENSITIVITY_FACTOR_);
    }

    float get_odr() const {
        return odr_;
    }

    auto get_rate_x() const {
        return std::get<0>(last_rate_);
    }

    auto get_rate_y() const {
        return std::get<1>(last_rate_);
    }

    auto get_rate_z() const {
        return std::get<2>(last_rate_);
    }

    auto get_update_time() const {
        return last_update_us_;
    }

protected:
    const uint8_t dev_addr_;
    I2C_Service* p_i2c_svc_;
    float range_factor_;
    float odr_;
    std::tuple<int16_t, int16_t, int16_t> raw_;
    std::tuple<int16_t, int16_t, int16_t> cal_;
    std::tuple<float, float, float> last_rate_;
    uint32_t last_update_us_;
};

} // namespace ctbot

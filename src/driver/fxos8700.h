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
 * @file    fxos8700.h
 * @brief   FXOS8700 sensor driver
 * @author  Timo Sandmann
 * @date    20.11.2022
 */

#pragma once

#include "i2c_service.h"

#include <cstdint>
#include <tuple>


namespace ctbot {
class FXOS8700 {
    static constexpr bool DEBUG_ { false };

protected:
    static constexpr uint8_t DEVICE_ID_ { 0xc7 };

    static constexpr uint8_t REGISTER_STATUS_ { 0 };
    static constexpr uint8_t REGISTER_OUT_X_MSB_ { 1 };
    static constexpr uint8_t REGISTER_OUT_X_LSB_ { 2 };
    static constexpr uint8_t REGISTER_OUT_Y_MSB_ { 3 };
    static constexpr uint8_t REGISTER_OUT_Y_LSB_ { 4 };
    static constexpr uint8_t REGISTER_OUT_Z_MSB_ { 5 };
    static constexpr uint8_t REGISTER_OUT_Z_LSB_ { 6 };
    static constexpr uint8_t REGISTER_SYSMOD_ { 0xb };
    static constexpr uint8_t REGISTER_WHO_AM_I_ { 0xd };
    static constexpr uint8_t REGISTER_XYZ_DATA_CFG_ { 0xe };
    static constexpr uint8_t REGISTER_CTRL_REG1_ { 0x2a };
    static constexpr uint8_t REGISTER_CTRL_REG2_ { 0x2b };
    static constexpr uint8_t REGISTER_CTRL_REG3_ { 0x2c };
    static constexpr uint8_t REGISTER_CTRL_REG4_ { 0x2d };
    static constexpr uint8_t REGISTER_CTRL_REG5_ { 0x2e };
    static constexpr uint8_t REGISTER_MSTATUS_ { 0x32 };
    static constexpr uint8_t REGISTER_MOUT_X_MSB_ { 0x33 };
    static constexpr uint8_t REGISTER_MOUT_X_LSB_ { 0x34 };
    static constexpr uint8_t REGISTER_MOUT_Y_MSB_ { 0x35 };
    static constexpr uint8_t REGISTER_MOUT_Y_LSB_ { 0x36 };
    static constexpr uint8_t REGISTER_MOUT_Z_MSB_ { 0x37 };
    static constexpr uint8_t REGISTER_MOUT_Z_LSB_ { 0x38 };
    static constexpr uint8_t REGISTER_MCTRL_REG1_ { 0x5b };
    static constexpr uint8_t REGISTER_MCTRL_REG2_ { 0x5c };
    static constexpr uint8_t REGISTER_MCTRL_REG3_ { 0x5d };

    static constexpr float ACCEL_MG_LSB_2G_ { 0.000244f };
    static constexpr float ACCEL_MG_LSB_4G_ { 0.000488f };
    static constexpr float ACCEL_MG_LSB_8G_ { 0.000976f };
    static constexpr float ACCEL_GRAVITY_EARTH_ { 9.80665f };
    static constexpr float MAG_UT_LSB_ { 0.1f };

public:
    enum class accel_range_t : uint8_t { ACCEL_RANGE_2G = 0, ACCEL_RANGE_4G = 1, ACCEL_RANGE_8G = 2 };

    enum class mag_osr_t : uint8_t {
        MAG_OSR_0, /**< Mag oversampling ratio = 0 */
        MAG_OSR_1, /**< Mag oversampling ratio = 1 */
        MAG_OSR_2, /**< Mag oversampling ratio = 2 */
        MAG_OSR_3, /**< Mag oversampling ratio = 3 */
        MAG_OSR_4, /**< Mag oversampling ratio = 4 */
        MAG_OSR_5, /**< Mag oversampling ratio = 5 */
        MAG_OSR_6, /**< Mag oversampling ratio = 6 */
        MAG_OSR_7, /**< Mag oversampling ratio = 7 */
    };

    enum class odr_t : uint8_t {
        ODR_800HZ, /**< 800Hz, only available in accel/mag-only modes */
        ODR_400HZ, /**< 400Hz, available in all modes */
        ODR_200HZ, /**< 200Hz, available in all modes */
        ODR_100HZ, /**< 100Hz, available in all modes */
        ODR_50HZ, /**< 50Hz, available in all modes */
        ODR_25HZ, /**< 25Hz, only available in hybrid mode */
        ODR_12_5HZ, /**< 12.5Hz, only available in accel/mag-only modes */
        ODR_6_25HZ, /**< 6.25Hz, available in all modes */
        ODR_3_125HZ, /**< 3.125Hz, only available in hybrid mode */
        ODR_1_5625HZ, /**< 3.125Hz, only available in accel/mag-only modes */
        ODR_0_7813HZ, /**< 0.7813Hz, only available in hybrid mode */
    };

    enum class sensor_mode_t : uint8_t { ACCEL_ONLY_MODE = 0b00, MAG_ONLY_MODE = 0b01, HYBRID_MODE = 0b11 };

    FXOS8700(I2C_Service* p_i2c_svc, uint8_t dev_addr = 0x1f);

    ~FXOS8700() = default;

    bool begin();

    bool calibrate();

    bool update();

    bool standby(bool standby) const;

    bool set_sensor_mode(sensor_mode_t mode);

    sensor_mode_t get_sensor_mode() const {
        return mode_;
    }

    bool set_accel_range(accel_range_t range);

    accel_range_t get_accel_range() const {
        return range_;
    }

    bool set_output_data_rate(odr_t rate);

    odr_t get_output_data_rate() const {
        return rate_;
    }

    bool set_mag_oversampling_ratio(mag_osr_t ratio);

    mag_osr_t get_mag_oversampling_ratio() const {
        return ratio_;
    }

    auto get_acc_x() const {
        return std::get<0>(last_accel_);
    }

    auto get_acc_y() const {
        return std::get<1>(last_accel_);
    }

    auto get_acc_z() const {
        return std::get<2>(last_accel_);
    }

    auto get_update_time() const {
        return last_update_us_;
    }


protected:
    enum class system_status_t : uint8_t {
        STANDBY = 0, /**< sysmod[1:0]. Standby status */
        WAKE = 1, /**< sysmod[1:0]. Wake status */
        SLEEP = 2 /**< sysmod[1:0]. Sleep status */
    };

    static constexpr odr_t ACCEL_MAG_ONLY_AVAILABLE_ODRS_[] = {
        odr_t::ODR_800HZ, /**< 800Hz, only available in accel/mag-only modes */
        odr_t::ODR_400HZ, /**< 400Hz, available in all modes */
        odr_t::ODR_200HZ, /**< 200Hz, available in all modes */
        odr_t::ODR_100HZ, /**< 100Hz, available in all modes */
        odr_t::ODR_50HZ, /**< 50Hz, available in all modes */
        odr_t::ODR_12_5HZ, /**< 12.5Hz, only available in accel/mag-only modes */
        odr_t::ODR_6_25HZ, /**< 6.25Hz, available in all modes */
        odr_t::ODR_1_5625HZ, /**< 3.125Hz, only available in accel/mag-only modes */
    };

    static constexpr odr_t HYBRID_AVAILABLE_ODRS_[] = {
        odr_t::ODR_400HZ, /**< 400Hz, available in all modes */
        odr_t::ODR_200HZ, /**< 200Hz, available in all modes */
        odr_t::ODR_100HZ, /**< 100Hz, available in all modes */
        odr_t::ODR_50HZ, /**< 50Hz, available in all modes */
        odr_t::ODR_25HZ, /**< 25Hz, only available in hybrid mode */
        odr_t::ODR_6_25HZ, /**< 6.25Hz, available in all modes */
        odr_t::ODR_3_125HZ, /**< 3.125Hz, only available in hybrid mode */
        odr_t::ODR_0_7813HZ, /**< 0.7813Hz, only available in hybrid mode */
    };

    static constexpr uint16_t ODR_DR_BITS_[] = {
        0x00, /**< dr=0b000. 800Hz accel/mag-only modes, 400Hz hyrbid mode */
        0x08, /**< dr=0b001. 400Hz accel/mag-only modes, 200Hz hyrbid mode */
        0x10, /**< dr=0b010. 200Hz accel/mag-only modes, 100Hz hyrbid mode */
        0x18, /**< dr=0b011. 100Hz accel/mag-only modes, 50Hz hyrbid mode */
        0x20, /**< dr=0b100. 50Hz accel/mag-only modes, 25Hz hyrbid mode */
        0x28, /**< dr=0b101. 12.5Hz accel/mag-only modes, 6.25Hz hyrbid mode */
        0x30, /**< dr=0b110. 6.25Hz accel/mag-only modes, 3.125Hz hyrbid mode */
        0x38, /**< dr=0b111. 1.5625Hz accel/mag-only modes, 0.7813Hz hyrbid mode */
    };

    const uint8_t dev_addr_;
    I2C_Service* p_i2c_svc_;
    sensor_mode_t mode_;
    float acc_range_factor_;
    accel_range_t range_;
    odr_t rate_;
    mag_osr_t ratio_;
    std::tuple<int16_t, int16_t, int16_t> accel_raw_;
    std::tuple<int16_t, int16_t, int16_t> accel_cal_;
    std::tuple<float, float, float> last_accel_;
    std::tuple<int16_t, int16_t, int16_t> mag_raw_;
    std::tuple<float, float, float> last_mag_;
    uint32_t last_update_us_;
};

} // namespace ctbot

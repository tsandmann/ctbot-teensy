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
 * @file    vl6180x.cpp
 * @brief   VL6180X sensor driver
 * @author  Timo Sandmann
 * @date    07.12.2019
 */

#include "vl6180x.h"
#include "ctbot.h"

#include <array>


namespace ctbot {
VL6180X::VL6180X(const uint8_t i2c_bus, const uint32_t i2c_freq, const uint8_t i2c_addr) : i2c_ { i2c_bus, i2c_addr, i2c_freq }, ptp_offset_ {}, scaling_ {} {}

FLASHMEM bool VL6180X::init() {
    if (!i2c_.init()) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print("VL6180X::init(): i2c_.init() failed.\r\n", true);
        }
        return false;
    }

    /* store part-to-part range offset so it can be adjusted if scaling is changed */
    if (i2c_.read_reg8(SYSRANGE_PART_TO_PART_RANGE_OFFSET_REG, ptp_offset_)) {
        return false;
    }

    uint8_t tmp;
    if (i2c_.read_reg8(SYSTEM_FRESH_OUT_OF_RESET_REG, tmp)) {
        return false;
    }
    if (tmp == 1) {
        scaling_ = 1;

        uint8_t ret {};
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x207), 1);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x208), 1);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x96), 0);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x97), 0xfd);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xe3), 0);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xe4), 4);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xe5), 2);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xe6), 1);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xe7), 3);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xf5), 2);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xd9), 5);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xdb), 0xce);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xdc), 3);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xdd), 0xf8);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x9f), 0);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xa3), 0x3c);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xb7), 0);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xbb), 0x3c);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xb2), 9);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xca), 9);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x198), 1);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x1b0), 0x17);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x1ad), 0);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0xff), 5);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x100), 5);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x199), 5);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x1a6), 0x1b);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x1ac), 0x3e);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x1a7), 0x1f);
        ret |= i2c_.write_reg8(static_cast<uint16_t>(0x30), 0);

        ret |= i2c_.write_reg8(SYSTEM_FRESH_OUT_OF_RESET_REG, 0);

        if (ret) {
            return false;
        }
    } else {
        /* sensor has already been initialized, so try to get scaling settings by reading registers */
        uint16_t s;
        if (i2c_.read_reg16(RANGE_SCALER_REG, s)) {
            return false;
        }

        if (s == SCALER_VALUES[3]) {
            scaling_ = 3;
        } else if (s == SCALER_VALUES[2]) {
            scaling_ = 2;
        } else {
            scaling_ = 1;
        }

        /* adjust the part-to-part range offset value read earlier to account for existing scaling. If the sensor was
         * already in 2x or 3x scaling mode, precision will be lost calculating the original (1x) offset, but this can
         * be resolved by resetting the sensor and MCU again */
        ptp_offset_ *= scaling_;
    }

    return true;
}

FLASHMEM bool VL6180X::configure_defaults() {
    uint8_t ret {};

    // readout__averaging_sample_period = 48
    ret |= i2c_.write_reg8(READOUT_AVERAGING_SAMPLE_PERIOD_REG, 0x30);

    // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to Table 14 in datasheet)
    ret |= i2c_.write_reg8(SYSALS_ANALOGUE_GAIN_REG, 0x46);

    // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
    ret |= i2c_.write_reg8(SYSRANGE_VHV_REPEAT_RATE_REG, 0xff);

    // sysals__integration_period = 99 (100 ms)
    // AN4545 incorrectly recommends writing to register 0x40; 0x63 should go in the lower byte, which is register 0x41.
    ret |= i2c_.write_reg16(SYSALS_INTEGRATION_PERIOD_REG, 0x63);

    // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
    ret |= i2c_.write_reg8(SYSRANGE_VHV_RECALIBRATE_REG, 1);

    // sysrange__intermeasurement_period = 9 (100 ms)
    ret |= i2c_.write_reg8(SYSRANGE_INTERMEASUREMENT_PERIOD_REG, 9);

    // sysals__intermeasurement_period = 49 (500 ms)
    ret |= i2c_.write_reg8(SYSALS_INTERMEASUREMENT_PERIOD_REG, 0x31);

    // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
    ret |= i2c_.write_reg8(SYSTEM_INTERRUPT_CONFIG_GPIO_REG, 0x24);

    // sysrange__max_convergence_time = 49 (49 ms)
    ret |= i2c_.write_reg8(SYSRANGE_MAX_CONVERGENCE_TIME_REG, 0x31);

    // disable interleaved mode
    ret |= i2c_.write_reg8(INTERLEAVED_MODE_ENABLE_REG, 0);

    // reset range scaling factor to 1x
    return set_scaling(1);
}

FLASHMEM bool VL6180X::set_scaling(const uint8_t scaling) {
    static constexpr uint8_t DEFAULT_CROSSTALK_VALID_HEIGHT { 20 };

    if (scaling < 1 || scaling > 3) {
        return false;
    }

    scaling_ = scaling;

    uint8_t ret {};
    ret |= i2c_.write_reg16(RANGE_SCALER_REG, SCALER_VALUES[scaling_]);

    // apply scaling on part-to-part offset
    ret |= i2c_.write_reg8(SYSRANGE_PART_TO_PART_RANGE_OFFSET_REG, ptp_offset_ / scaling_);

    // apply scaling on CrossTalkValidHeight
    ret |= i2c_.write_reg8(SYSRANGE_CROSSTALK_VALID_HEIGHT_REG, DEFAULT_CROSSTALK_VALID_HEIGHT / scaling_);

    // enable early convergence estimate only at 1x scaling
    uint8_t rce;
    ret |= i2c_.read_reg8(SYSRANGE_RANGE_CHECK_ENABLES_REG, rce);
    ret |= i2c_.write_reg8(SYSRANGE_RANGE_CHECK_ENABLES_REG, (rce & 0xfe) | (scaling_ == 1));

    return ret == 0;
}

FLASHMEM bool VL6180X::set_address(const uint8_t addr) {
    if (addr > 127) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print("VL6180X::set_address(): invalid address.\r\n", true);
        }
        return false;
    }
    if (i2c_.write_reg8(I2C_ADDRESS_REG, addr)) {
        if (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print("VL6180X::set_address(): write() failed.\r\n", true);
        }
        return false;
    }
    i2c_.set_address(addr);

    return true;
}

FLASHMEM bool VL6180X::start_continuous(const uint32_t period_ms) const {
    int16_t period_reg { static_cast<int16_t>(period_ms / 10U - 1U) };
    period_reg = std::max<int16_t>(0, period_reg);
    period_reg = std::min<int16_t>(period_reg, 254);

    uint8_t ret {};
    ret |= i2c_.write_reg8(SYSRANGE_INTERMEASUREMENT_PERIOD_REG, period_reg);
    ret |= i2c_.write_reg8(SYSRANGE_START_REG, 3);

    return ret == 0;
}

bool VL6180X::get_dist_range(uint8_t& range_mm) const {
    uint8_t data;
    if (!i2c_.read_reg8(RESULT_INTERRUPT_STATUS_REG, data)) {
        if (data & 4) {
            if (i2c_.read_reg8(RESULT_RANGE_VAL_REG, range_mm) && DEBUG_) {
                CtBot::get_instance().get_comm()->debug_print("VL6180X::get_dist_range(): i2c error 2\r\n", true);
            }
            if (i2c_.write_reg8(SYSTEM_INTERRUPT_CLEAR_REG, 1) && DEBUG_) {
                CtBot::get_instance().get_comm()->debug_print("VL6180X::get_dist_range(): i2c error 3\r\n", true);
            }
            return true;
        }
    } else if (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_print("VL6180X::get_dist_range(): i2c error 1\r\n", true);
    }

    return false;
}

} // namespace ctbot

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
 *
 * The register map and the timing encoding formulars are taken from
 * https://github.com/pololu/vl53l0x-arduino and the following applies
 * to these source code parts:
 * --------------------------------------------------------------------
 * Copyright (c) 2017 Pololu Corporation. For more information, see
 * https://www.pololu.com.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file    vl53l0x.h
 * @brief   VL53L0X sensor driver
 * @author  Timo Sandmann
 * @date    07.12.2019
 */

#pragma once

#include "i2c_service.h"

#include <cstdint>


namespace ctbot {
class VL53L0X {
    static constexpr bool DEBUG_ { false };
    static constexpr uint16_t MIN_DISTANCE_ { 40 }; // mm
    static constexpr uint16_t MAX_DISTANCE_ { 900 }; // mm

    struct SequenceStepEnables {
        bool tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts {
        uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
        uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
    };

    static constexpr uint8_t DEFAULT_I2C_ADDR { 0x29 };

    friend class SimConnection;
    static constexpr uint8_t SYSRANGE_START_REG { 0 };
    static constexpr uint8_t SEQUENCE_CONFIG_REG { 1 };
    static constexpr uint8_t INTERMEASUREMENT_PERIOD_REG { 4 };
    static constexpr uint8_t RESULT_INTERRUPT_STATUS_REG { 0x13 };
    static constexpr uint8_t RESULT_RANGE_STATUS_REG { 0x14 };
    static constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG { 0x44 };
    static constexpr uint8_t TIMEOUT_MACROP_REG { 0x46 };
    static constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD_REG { 0x50 };
    static constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_REG_HI { 0x51 };
    static constexpr uint8_t FINAL_RANGE_CONFIG_VCSEL_PERIOD_REG { 0x70 };
    static constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_REG_HI { 0x71 };
    static constexpr uint8_t INTERRUPT_CONFIG_GPIO_REG { 0xa };
    static constexpr uint8_t INTERRUPT_CLEAR_REG { 0xb };
    static constexpr uint8_t REF_EN_START_SELECT_REG { 0xb6 };
    static constexpr uint8_t SPAD_NUM_REQUESTED_REF_SPAD_REG { 0x4e };
    static constexpr uint8_t SPAD_REF_EN_START_OFFSET_REG { 0x4f };
    static constexpr uint8_t MSRC_CONFIG_REG { 0x60 };
    static constexpr uint8_t MUX_ACTIVE_HIGH_REG { 0x84 };
    static constexpr uint8_t VOLTAGE_MODE_REG { 0x89 };
    static constexpr uint8_t I2C_ADDRESS_REG { 0x8a };
    static constexpr uint8_t SPAD_ENABLES_REF_0_REG { 0xb0 };
    static constexpr uint8_t MODEL_ID_REG { 0xc0 };
    static constexpr uint8_t OSC_CALIBRATE_VAL_REG { 0xf8 };

    static constexpr uint8_t vcsel_period_decode(const uint8_t reg_val) {
        return (reg_val + 1U) << 1U;
    }

    static constexpr uint32_t calc_macro_period(const uint8_t vcsel_period_pclks) {
        return (2304UL * 1655UL * static_cast<uint32_t>(vcsel_period_pclks) + 500UL) / 1000UL;
    }

    static constexpr uint32_t timeout_mclks_to_us(const uint16_t timeout_period_mclks, const uint8_t vcsel_period_pclks) {
        uint32_t macro_period_ns { calc_macro_period(vcsel_period_pclks) };
        return (timeout_period_mclks * macro_period_ns + 500U) / 1000U;
    }

    static constexpr uint32_t timeout_us_to_mclks(const uint32_t timeout_period_us, const uint8_t vcsel_period_pclks) {
        const uint32_t macro_period_ns { calc_macro_period(vcsel_period_pclks) };
        return ((timeout_period_us * 1000UL) + (macro_period_ns / 2U)) / macro_period_ns;
    }

    static constexpr uint16_t timeout_decode(uint16_t reg_val) {
        /* format: (LSByte * 2^MSByte) + 1 */
        return static_cast<uint16_t>((reg_val & 0x00ff) << static_cast<uint16_t>((reg_val & 0xff00) >> 8U)) + 1U;
    }

    static constexpr uint16_t timeout_encode(const uint32_t timeout_mclks) {
        /* format: (LSByte * 2^MSByte) + 1 */
        if (timeout_mclks > 0U) {
            uint32_t ls_byte { timeout_mclks - 1U };
            uint16_t ms_byte {};
            while ((ls_byte & 0xffffff00) > 0U) {
                ls_byte >>= 1U;
                ms_byte++;
            }

            return (ms_byte << 8U) | (ls_byte & 0xff);
        }

        return 0;
    }

    I2C_Service i2c_;
    uint8_t i2c_addr_;
    uint32_t timing_budget_us_;
    uint8_t stop_variable_;

    FLASHMEM bool get_spad_info(uint8_t& count, bool& type_is_aperture) const;
    FLASHMEM uint32_t get_measurement_timing_budget() const;
    FLASHMEM bool set_measurement_timing_budget(const uint32_t budget_us) const;
    FLASHMEM bool get_sequence_step_enables(SequenceStepEnables& enables) const;
    FLASHMEM bool get_sequence_step_timeouts(const SequenceStepEnables& enables, SequenceStepTimeouts& timeouts) const;
    FLASHMEM uint8_t get_vcsel_pulse_period(const bool final) const;
    FLASHMEM bool perform_single_ref_calibration(const uint8_t vhv_init_byte) const;
    FLASHMEM bool set_signal_rate_limit(const float limit_mcps) const;

    uint8_t read_reg8(const uint8_t reg, uint8_t& data) const {
        return i2c_.read_reg(i2c_addr_, reg, data);
    }

    uint8_t read_reg16(const uint8_t reg, uint16_t& data) const {
        return i2c_.read_reg(i2c_addr_, reg, data);
    }

    uint8_t read_bytes(const uint8_t addr, uint8_t* p_data, const uint8_t length) const {
        return i2c_.read_bytes(i2c_addr_, addr, p_data, length);
    }

    uint8_t write_reg8(const uint8_t reg, const uint8_t value) const {
        return i2c_.write_reg(i2c_addr_, reg, value);
    }

    uint8_t write_reg16(const uint8_t reg, const uint16_t value) const {
        return i2c_.write_reg(i2c_addr_, reg, value);
    }

    uint8_t write_reg32(const uint8_t reg, const uint32_t value) const {
        return i2c_.write_reg(i2c_addr_, reg, value);
    }

    uint8_t write_bytes(const uint8_t addr, const uint8_t* p_data, const uint8_t length) const {
        return i2c_.write_bytes(i2c_addr_, addr, p_data, length);
    }

    uint8_t set_bit(const uint8_t reg, const uint8_t bit, const bool value) const {
        return i2c_.set_bit(i2c_addr_, reg, bit, value);
    }

public:
    FLASHMEM VL53L0X(const uint8_t i2c_bus, const uint32_t i2c_freq, const uint8_t i2c_addr = DEFAULT_I2C_ADDR); // FIXME: pass I2C_Service

    bool init();

    FLASHMEM bool set_address(const uint8_t addr);

    FLASHMEM bool start_continuous(const uint32_t period_ms) const;

    bool get_dist_range(uint16_t& range_mm) const;
};
} // namespace ctbot

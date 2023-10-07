/*
 * This file is part of the ct-Bot teensy framework.
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
 * The register map is taken from
 * https://github.com/pololu/vl6180x-arduino and the following applies
 * to these source code parts:
 * --------------------------------------------------------------------
 * Copyright (c) 2016 Pololu Corporation. For more information, see
 * http://www.pololu.com.
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
 * @file    vl6180x.h
 * @brief   VL6180X sensor driver
 * @author  Timo Sandmann
 * @date    07.12.2019
 */

#pragma once

#include "i2c_service.h"

#include <cstdint>


namespace ctbot {
class VL6180X {
    static constexpr bool DEBUG_ { false };

    static constexpr uint8_t DEFAULT_I2C_ADDR { 0x29 };

    friend class SimConnection;
    static constexpr uint16_t SYSTEM_INTERRUPT_CONFIG_GPIO_REG { 0x14 };
    static constexpr uint16_t SYSTEM_INTERRUPT_CLEAR_REG { 0x15 };
    static constexpr uint16_t SYSTEM_FRESH_OUT_OF_RESET_REG { 0x16 };
    static constexpr uint16_t SYSRANGE_START_REG { 0x18 };
    static constexpr uint16_t SYSRANGE_CROSSTALK_VALID_HEIGHT_REG { 0x21 };
    static constexpr uint16_t SYSRANGE_PART_TO_PART_RANGE_OFFSET_REG { 0x24 };
    static constexpr uint16_t SYSRANGE_RANGE_CHECK_ENABLES_REG { 0x2d };
    static constexpr uint16_t SYSRANGE_VHV_REPEAT_RATE_REG { 0x31 };
    static constexpr uint16_t SYSALS_INTEGRATION_PERIOD_REG { 0x40 };
    static constexpr uint16_t RANGE_SCALER_REG { 0x96 };
    static constexpr uint16_t SYSRANGE_INTERMEASUREMENT_PERIOD_REG { 0x1b };
    static constexpr uint16_t SYSRANGE_MAX_CONVERGENCE_TIME_REG { 0x1c };
    static constexpr uint16_t SYSRANGE_VHV_RECALIBRATE_REG { 0x2e };
    static constexpr uint16_t SYSALS_INTERMEASUREMENT_PERIOD_REG { 0x3e };
    static constexpr uint16_t SYSALS_ANALOGUE_GAIN_REG { 0x3f };
    static constexpr uint16_t RESULT_INTERRUPT_STATUS_REG { 0x4f };
    static constexpr uint16_t RESULT_RANGE_VAL_REG { 0x62 };
    static constexpr uint16_t READOUT_AVERAGING_SAMPLE_PERIOD_REG { 0x10a };
    static constexpr uint16_t I2C_ADDRESS_REG { 0x212 };
    static constexpr uint16_t INTERLEAVED_MODE_ENABLE_REG { 0x2a3 };

    static constexpr uint16_t SCALER_VALUES[] = { 0, 253, 127, 84 };

    I2C_Service& i2c_;
    uint8_t i2c_addr_;
    uint8_t ptp_offset_;
    uint8_t scaling_;

    FLASHMEM bool set_scaling(const uint8_t scaling);

    uint8_t read_reg8(const uint16_t reg, uint8_t& data) const {
        return static_cast<uint8_t>(i2c_.read_reg(i2c_addr_, reg, data));
    }

    uint8_t read_reg16(const uint16_t reg, uint16_t& data) const {
        return static_cast<uint8_t>(i2c_.read_reg(i2c_addr_, reg, data));
    }

    uint8_t write_reg8(const uint16_t reg, const uint8_t value) const {
        return static_cast<uint8_t>(i2c_.write_reg(i2c_addr_, reg, value));
    }

    uint8_t write_reg16(const uint16_t reg, const uint16_t value) const {
        return static_cast<uint8_t>(i2c_.write_reg(i2c_addr_, reg, value));
    }

public:
    FLASHMEM VL6180X(I2C_Service& i2c_svc, uint8_t i2c_addr = DEFAULT_I2C_ADDR);

    FLASHMEM bool init();

    FLASHMEM bool set_address(const uint8_t addr);

    FLASHMEM bool configure_defaults();

    FLASHMEM bool start_continuous(const uint32_t period_ms) const;

    bool get_dist_range(uint8_t& range_mm) const;
};
} // namespace ctbot

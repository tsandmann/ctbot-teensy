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
 * @file    i2c_wrapper.h
 * @brief   Wrapper for I2C interface
 * @author  Timo Sandmann
 * @date    17.08.2019
 */

#pragma once

#include "arduino_freertos.h"

#include <cstdint>
#include <mutex>
#include <array>


namespace arduino {
class I2CT4;
} // namespace arduino

namespace ctbot {

class I2C_Wrapper {
    static constexpr bool DEBUG_ { false };

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
    using I2C_t = arduino::I2CT4;
#else
    using I2C_t = arduino::TwoWire;
#endif

    I2C_t* p_i2c_;
    uint8_t bus_;
    uint8_t addr_;
    uint32_t freq_;
    static std::array<std::timed_mutex, 4> mutex_;

    uint32_t get_freq_internal() const;

public:
    I2C_Wrapper(const uint8_t bus, const uint8_t addr, const uint32_t freq);

    I2C_Wrapper(const uint8_t bus, const uint8_t addr) : I2C_Wrapper { bus, addr, 100'000 } {}

    I2C_Wrapper(const uint8_t bus) : I2C_Wrapper { bus, 255, 100'000 } {}

    uint8_t get_bus() const {
        return bus_;
    }

    bool init();

    bool init(const uint8_t bus_id, const uint32_t freq);

    uint32_t get_freq() const {
        return freq_;
    }

    uint8_t get_address() const {
        return addr_;
    }

    void set_address(const uint8_t addr);

    uint8_t read_reg8(const uint8_t reg, uint8_t& data) const;
    uint8_t read_reg8(const uint16_t reg, uint8_t& data) const;
    uint8_t read_reg16(const uint8_t reg, uint16_t& data) const;
    uint8_t read_reg32(const uint8_t reg, uint32_t& data) const;
    uint8_t read_bytes(const uint8_t addr, void* p_data, const uint8_t length) const;

    uint8_t write_reg8(const uint8_t reg, const uint8_t value) const;
    uint8_t write_reg8(const uint16_t reg, const uint8_t value) const;
    uint8_t write_reg16(const uint8_t reg, const uint16_t value) const;
    uint8_t write_reg32(const uint8_t reg, const uint32_t value) const;
    uint8_t write_bytes(const uint8_t addr, const void* p_data, const uint8_t length) const;

    uint8_t set_bit(const uint8_t reg, const uint8_t bit, const bool value) const;

    bool test(const uint8_t addr);
};

} // namespace ctbot

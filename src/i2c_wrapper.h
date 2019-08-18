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

#include "arduino_fixed.h"

#include <cstdint>


namespace ctbot {

class I2C_Wrapper {
    static arduino::TwoWire* p_i2c_;
    static uint8_t bus_;
    static uint8_t addr_;

public:
    static uint8_t get_bus() {
        return bus_;
    }
    static bool set_bus(const uint8_t bus_id, const uint16_t freq = 100);

    static uint8_t get_address() {
        return addr_;
    }
    static void set_address(const uint8_t addr);

    static uint8_t read_reg8(const uint8_t reg, uint8_t& data);
    static uint8_t read_reg16(const uint8_t reg, uint16_t& data);
    static uint8_t read_reg32(const uint8_t reg, uint32_t& data);

    static uint8_t write_reg8(const uint8_t reg, const uint8_t value);
    static uint8_t write_reg16(const uint8_t reg, const uint16_t value);
    static uint8_t write_reg32(const uint8_t reg, const uint32_t value);
};

} // namespace ctbot

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
 * @file    i2c_wrapper.cpp
 * @brief   Wrapper for I2C interface
 * @author  Timo Sandmann
 * @date    17.08.2019
 */

#include "i2c_wrapper.h"
#include "ctbot_config.h"


namespace ctbot {
arduino::TwoWire* I2C_Wrapper::p_i2c_ {};
uint8_t I2C_Wrapper::bus_ {};
uint8_t I2C_Wrapper::addr_ {};

bool I2C_Wrapper::set_bus(const uint8_t bus_id, const uint16_t freq) {
    switch (bus_id) {
        case 0: {
            p_i2c_ = &arduino::Wire;
            p_i2c_->begin();
            p_i2c_->setSDA(CtBotConfig::I2C0_PIN_SDA);
            p_i2c_->setSCL(CtBotConfig::I2C0_PIN_SCL);
            p_i2c_->setClock(freq * 1000U);
            break;
        }

            // case 1: {
            //     p_i2c_ = &arduino::Wire1;
            //     p_i2c_->begin();
            //     p_i2c_->setSDA(CtBotConfig::I2C1_PIN_SDA);
            //     p_i2c_->setSCL(CtBotConfig::I2C1_PIN_SCL);
            //     p_i2c_->setClock(freq * 1000U);
            //     break;
            // }

        case 2: {
            p_i2c_ = &arduino::Wire2;
            p_i2c_->begin();
            p_i2c_->setSDA(CtBotConfig::I2C2_PIN_SDA);
            p_i2c_->setSCL(CtBotConfig::I2C2_PIN_SCL);
            p_i2c_->setClock(freq * 1000U);
            break;
        }


            // case 3: { //FIXME: activate I2C_3
            //     p_i2c_ = &arduino::Wire3;
            //     p_i2c_->begin();
            //     p_i2c_->setSDA(CtBotConfig::I2C3_PIN_SDA);
            //     p_i2c_->setSCL(CtBotConfig::I2C3_PIN_SCL);
            //     p_i2c_->setClock(freq * 1000U);
            //     break;
            // }

        default: {
            p_i2c_ = nullptr;
            return false;
        }
    }

    return true;
}

void I2C_Wrapper::set_address(const uint8_t addr) {
    addr_ = addr;
}

uint8_t I2C_Wrapper::read_reg8(const uint8_t reg, uint8_t& data) {
    if (!p_i2c_) {
        return 4;
    }

    p_i2c_->beginTransmission(addr_);
    p_i2c_->write(reg);
    const uint8_t status { p_i2c_->endTransmission() };
    if (status) {
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 1U) != 1U) {
        return 4;
    }
    while (p_i2c_->available() < 1) {
    }

    const auto x { p_i2c_->read() };
    if (x < 0) {
        return 4;
    }
    data = static_cast<uint8_t>(x);

    return 0;
}

uint8_t I2C_Wrapper::read_reg16(const uint8_t reg, uint16_t& data) {
    if (!p_i2c_) {
        return 4;
    }

    p_i2c_->beginTransmission(addr_);
    p_i2c_->write(reg);
    const uint8_t status { p_i2c_->endTransmission() };
    if (status) {
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 2U) != 2U) {
        return 4;
    }
    while (p_i2c_->available() < 2) {
    }
    auto x { p_i2c_->read() };
    if (x < 0) {
        return 4;
    }
    data = static_cast<uint16_t>(x) << 8; // high byte

    x = p_i2c_->read();
    if (x < 0) {
        return 4;
    }
    data |= static_cast<uint8_t>(x); // low byte

    return 0;
}

uint8_t I2C_Wrapper::read_reg32(const uint8_t reg, uint32_t& data) {
    if (!p_i2c_) {
        return 4;
    }

    p_i2c_->beginTransmission(addr_);
    p_i2c_->write(reg);
    const uint8_t status { p_i2c_->endTransmission() };
    if (status) {
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 4U) != 4U) {
        return 4;
    }
    while (p_i2c_->available() < 4) {
    }

    auto x { p_i2c_->read() };
    if (x < 0) {
        return 4;
    }
    data = static_cast<uint32_t>(x) << 24; // highest byte

    x = p_i2c_->read();
    if (x < 0) {
        return 4;
    }
    data |= static_cast<uint32_t>(x) << 16;

    x = p_i2c_->read();
    if (x < 0) {
        return 4;
    }
    data |= static_cast<uint32_t>(x) << 8;

    x = p_i2c_->read();
    if (x < 0) {
        return 4;
    }
    data |= static_cast<uint8_t>(x); // lowest byte

    return 0;
}

uint8_t I2C_Wrapper::write_reg8(const uint8_t reg, const uint8_t value) {
    if (!p_i2c_) {
        return 4;
    }

    p_i2c_->beginTransmission(addr_);
    p_i2c_->write(reg);
    p_i2c_->write(value);
    return p_i2c_->endTransmission();
}


uint8_t I2C_Wrapper::write_reg16(const uint8_t reg, const uint16_t value) {
    if (!p_i2c_) {
        return 4;
    }

    p_i2c_->beginTransmission(addr_);
    p_i2c_->write(reg);
    p_i2c_->write((value >> 8) & 0xff); // high byte
    p_i2c_->write(value & 0xff); // low byte
    return p_i2c_->endTransmission();
}


uint8_t I2C_Wrapper::write_reg32(const uint8_t reg, const uint32_t value) {
    if (!p_i2c_) {
        return 4;
    }

    p_i2c_->beginTransmission(addr_);
    p_i2c_->write(reg);
    p_i2c_->write((value >> 24) & 0xff); // highest byte
    p_i2c_->write((value >> 16) & 0xff);
    p_i2c_->write((value >> 8) & 0xff);
    p_i2c_->write(value & 0xff); // lowest byte
    return p_i2c_->endTransmission();
}


} // namespace ctbot

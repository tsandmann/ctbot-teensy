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

#include <chrono>
#include <thread>


namespace ctbot {

decltype(I2C_Wrapper::mutex_) I2C_Wrapper::mutex_;

I2C_Wrapper::I2C_Wrapper(const uint8_t bus, const uint8_t addr, const uint32_t freq) : p_i2c_ {}, bus_ { bus }, addr_ { addr }, freq_ { freq } {}

bool I2C_Wrapper::init() {
    return init(bus_, freq_);
}

bool I2C_Wrapper::init(const uint8_t bus_id, const uint32_t freq) {
    if (bus_id > mutex_.size()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    bus_ = bus_id;
    freq_ = freq;

    uint8_t pin_sda, pin_scl;
    switch (bus_) {
        case 0: {
            p_i2c_ = &arduino::Wire;
            pin_sda = CtBotConfig::I2C0_PIN_SDA;
            pin_scl = CtBotConfig::I2C0_PIN_SCL;
            break;
        }

        case 1: {
            p_i2c_ = &arduino::Wire1;
            pin_sda = CtBotConfig::I2C1_PIN_SDA;
            pin_scl = CtBotConfig::I2C1_PIN_SCL;
            break;
        }

        case 2: {
            p_i2c_ = &arduino::Wire2;
            pin_sda = CtBotConfig::I2C2_PIN_SDA;
            pin_scl = CtBotConfig::I2C2_PIN_SCL;
            break;
        }

#if defined TwoWireKinetis_h && defined WIRE_IMPLEMENT_WIRE3
        case 3: {
            p_i2c_ = &arduino::Wire3;
            pin_sda = CtBotConfig::I2C3_PIN_SDA;
            pin_scl = CtBotConfig::I2C3_PIN_SCL;
            break;
        }
#endif

        default: {
            p_i2c_ = nullptr;
            return false;
        }
    }

    p_i2c_->begin();
    p_i2c_->setSDA(pin_sda);
    p_i2c_->setSCL(pin_scl);
    p_i2c_->setClock(freq_);

    freq_ = get_freq_internal();

    return true;
}

uint32_t I2C_Wrapper::get_freq_internal() const {
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
    return freq_;
#else
    switch (bus_) {
        case 0: {
            switch (KINETIS_I2C0.F) {
                case 0x2C: return 104'000;
                case 0x1C: return 416'000;
                case 0x12: return 938'000;
            }
            break;
        }
        case 1: {
            switch (KINETIS_I2C1.F) {
                case 0x2C: return 104'000;
                case 0x1C: return 416'000;
                case 0x12: return 938'000;
            }
            break;
        }
        case 2: {
            switch (KINETIS_I2C2.F) {
                case 0x2C: return 104'000;
                case 0x1C: return 416'000;
                case 0x12: return 938'000;
            }
            break;
        }
        case 3: {
            switch (KINETIS_I2C3.F) {
                case 0x2C: return 104'000;
                case 0x1C: return 416'000;
                case 0x12: return 938'000;
            }
            break;
        }
    }
#endif
    return 0;
}

void I2C_Wrapper::set_address(const uint8_t addr) {
    addr_ = addr;
}

uint8_t I2C_Wrapper::read_reg8(const uint8_t reg, uint8_t& data) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): write() failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(0U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::read_reg8(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 1U, 1U) != 1U) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): requestFrom() failed.");
        }
        return 4;
    }

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    const auto x { p_i2c_->read() };
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): read() failed.");
        }
        return 4;
    }
    data = static_cast<uint8_t>(x);

    return 0;
}

uint8_t I2C_Wrapper::read_reg8(const uint16_t reg, uint8_t& data) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg >> 8) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): write(high) failed.");
        }
        return 4;
    }
    if (p_i2c_->write(reg & 0xff) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): write(low) failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(0U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::read_reg8(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 1U, 1U) != 1U) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): requestFrom() failed.");
        }
        return 4;
    }

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg8(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    const auto x { p_i2c_->read() };
    if (x < 0) {
        return 4;
    }
    data = static_cast<uint8_t>(x);

    return 0;
}

uint8_t I2C_Wrapper::read_reg16(const uint8_t reg, uint16_t& data) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): write() failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(0U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::read_reg16(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 2U, 1U) != 2U) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): requestFrom() failed.");
        }
        return 4;
    }
    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    auto x { p_i2c_->read() };
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): read() failed.");
        }
        return 4;
    }
    data = static_cast<uint16_t>(x) << 8; // high byte

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    x = p_i2c_->read();
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg16(): read() failed.");
        }
        return 4;
    }
    data |= static_cast<uint8_t>(x); // low byte

    return 0;
}

uint8_t I2C_Wrapper::read_reg32(const uint8_t reg, uint32_t& data) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): write() failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(0U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::read_reg32(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
        return status;
    }

    if (p_i2c_->requestFrom(addr_, 4U, 1U) != 4U) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): requestFrom() failed.");
        }
        return 4;
    }

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    auto x { p_i2c_->read() };
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): read() failed.");
        }
        return 4;
    }
    data = static_cast<uint32_t>(x) << 24; // most significant byte

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    x = p_i2c_->read();
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): read() failed.");
        }
        return 4;
    }
    data |= static_cast<uint32_t>(x) << 16;

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    x = p_i2c_->read();
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): read() failed.");
        }
        return 4;
    }
    data |= static_cast<uint32_t>(x) << 8;

    while (p_i2c_->available() == 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }
    x = p_i2c_->read();
    if (x < 0) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_reg32(): read() failed.");
        }
        return 4;
    }
    data |= static_cast<uint8_t>(x); // least significant byte

    return 0;
}

uint8_t I2C_Wrapper::read_bytes(const uint8_t addr, void* p_data, const uint8_t length) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_bytes(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(addr) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_bytes(): write() failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(0U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::read_bytes(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
        return status;
    }

    if (p_i2c_->requestFrom(addr_, length, 1U) != length) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_bytes(): requestFrom() failed.");
        }
        return 4;
    }
    while (p_i2c_->available() < length) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_bytes(): waiting for requested data.");
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1ms);
    }

    if (p_i2c_->readBytes(static_cast<uint8_t*>(p_data), length) != length) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::read_bytes(): readBytes() failed.");
        }
        return 4;
    }

    return 0;
}

uint8_t I2C_Wrapper::write_reg8(const uint8_t reg, const uint8_t value) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): write() 1 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(value) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): write() 2 failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(1U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::write_reg8(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
    }
    return status;
}

uint8_t I2C_Wrapper::write_reg8(const uint16_t reg, const uint8_t value) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg >> 8) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): write() 1 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(reg & 0xff) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): write() 2 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(value) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg8(): write() 3 failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(1U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::write_reg8(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
    }
    return status;
}

uint8_t I2C_Wrapper::write_reg16(const uint8_t reg, const uint16_t value) const {
    // arduino::Serial.print("I2C_Wrapper::write_reg16(0x");
    // arduino::Serial.print(reg, 16);
    // arduino::Serial.print(", 0x");
    // arduino::Serial.print(value, 16);
    // arduino::Serial.print(") addr=0x");
    // arduino::Serial.println(addr_, 16);

    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg16(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg16(): write() 1 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(value >> 8) != 1) { // high byte
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg16(): write() 2 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(value & 0xff) != 1) { // low byte
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg16(): write() 3 failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(1U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::write_reg16(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
    }
    return status;
}

uint8_t I2C_Wrapper::write_reg32(const uint8_t reg, const uint32_t value) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg32(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(reg) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg32(): write() 1 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(value >> 24) != 1) { // most significant byte
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg32(): write() 2 failed.");
        }
        return 4;
    }
    if (p_i2c_->write((value >> 16) & 0xff) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg32(): write() 3 failed.");
        }
        return 4;
    }
    if (p_i2c_->write((value >> 8) & 0xff) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg32(): write() 4 failed.");
        }
        return 4;
    }
    if (p_i2c_->write(value & 0xff) != 1) { // least significant byte
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_reg32(): write() 5 failed.");
        }
        return 4;
    }
    const uint8_t status { p_i2c_->endTransmission(1U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::write_reg32(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
    }
    return status;
}

uint8_t I2C_Wrapper::write_bytes(const uint8_t addr, const void* p_data, const uint8_t length) const {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_bytes(): p_i2c_ not initialized.");
        }
        return 4;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    if (p_i2c_->write(addr) != 1) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_bytes(): write() 1 failed.");
        }
        return 4;
    }

    if (p_i2c_->write(static_cast<const uint8_t*>(p_data), length) != length) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::write_bytes(): write() 2 failed.");
        }
        return 4;
    }

    const uint8_t status { p_i2c_->endTransmission(1U) };
    if (status) {
        if (DEBUG_) {
            arduino::Serial.print("I2C_Wrapper::write_bytes(): endTransmission() failed: ");
            arduino::Serial.println(status, 10);
        }
    }
    return status;
}

uint8_t I2C_Wrapper::set_bit(const uint8_t reg, const uint8_t bit, const bool value) const {
    uint8_t data;
    if (read_reg8(reg, data)) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::set_bit(): read_reg8() failed.");
        }
        return 4;
    }

    value ? data |= 1 << bit : data &= ~(1 << bit);

    if (write_reg8(reg, data)) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::set_bit(): write_reg8() failed.");
        }
        return 4;
    }
    return 0;
}

bool I2C_Wrapper::test(const uint8_t addr) {
    if (!p_i2c_) {
        if (DEBUG_) {
            arduino::Serial.println("I2C_Wrapper::test(): p_i2c_ not initialized.");
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_[bus_]);

    addr_ = addr;
    p_i2c_->setClock(freq_);
    p_i2c_->beginTransmission(addr_);
    return p_i2c_->endTransmission(1U) == 0;

    // Address  R/W Description
    // 0000 000 0   General call address
    // 0000 000 1   START byte
    // 0000 001 X   CBUS address
    // 0000 010 X   reserved - different bus format
    // 0000 011 X   reserved - future purposes
    // 0000 1XX X   High Speed master code
    // 1111 1XX X   reserved - future purposes
    // 1111 0XX X   10-bit slave addressing
}

} // namespace ctbot

/*
 * Copyright (c) 2018 Timo Sandmann
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
 * @file    Wire.h
 * @brief   Wrapper aroung Arduino stuff to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <map>
#include <iostream>


typedef struct {
    volatile uint8_t A1;
    volatile uint8_t F;
    volatile uint8_t C1;
    volatile uint8_t S;
    volatile uint8_t D;
    volatile uint8_t C2;
    volatile uint8_t FLT;
    volatile uint8_t RA;
    volatile uint8_t SMB;
    volatile uint8_t A2;
    volatile uint8_t SLTH;
    volatile uint8_t SLTL;
} KINETIS_I2C_t;


class TwoWire {
    static constexpr bool DEBUG_ { false };

    std::map<uint8_t /*addr*/, std::vector<uint8_t>> data_;
    std::map<uint8_t /*addr*/, uint8_t> addr_width_;
    uint8_t addr_;
    uint16_t reg_;
    uint8_t mode_;
    int available_;

public:
    TwoWire() : addr_ {}, reg_ {}, mode_ {}, available_ { -1 } {}

    void begin() {}
    void setSDA(uint8_t) {}
    void setSCL(uint8_t) {}
    void setClock(uint32_t) {}

    void beginTransmission(uint8_t address) {
        if (address >= 8 && address < 120) {
            addr_ = address;
            mode_ = 1; // reg addr low follows
            if (DEBUG_) {
                std::cout << "TwoWire::beginTransmission(): address set to 0x" << std::hex << static_cast<uint32_t>(addr_) << std::dec << "\n";
            }
        } else {
            if (DEBUG_) {
                std::cout << "TwoWire::beginTransmission(): invalid address: 0x" << std::hex << static_cast<uint32_t>(address) << std::dec << "\n";
            }
        }
    }

    uint8_t endTransmission([[maybe_unused]] uint8_t sendStop = 1) {
        mode_ = 3; // data follows
        return 0;
    }

    virtual size_t write(uint8_t data) {
        if (mode_ == 1) {
            reg_ = data;
            if (DEBUG_) {
                std::cout << "TwoWire::write(): mode=" << static_cast<uint32_t>(mode_) << " reg_=0x" << std::hex << static_cast<uint32_t>(reg_) << std::dec
                          << "\n";
            }
            if (addr_width_[addr_] == 2) {
                mode_ = 2;
            } else {
                mode_ = 3;
            }
        } else if (mode_ == 2) {
            reg_ |= data << 8;
            if (DEBUG_) {
                std::cout << "TwoWire::write(): mode=" << static_cast<uint32_t>(mode_) << " reg_=0x" << std::hex << static_cast<uint32_t>(reg_) << std::dec
                          << "\n";
            }
            mode_ = 3;
        } else if (mode_ == 3) {
            auto& data_vec { data_[addr_] };
            if (data_vec.size() < 65'535) {
                data_vec.resize(65'535, 0);
                if (DEBUG_) {
                    std::cout << "TwoWire::write(): vector resized to 65535.\n";
                }
            }
            if (DEBUG_) {
                std::cout << "TwoWire::write(): [0x" << std::hex << static_cast<uint32_t>(reg_) << "] = 0x" << static_cast<uint32_t>(data) << std::dec << "\n";
            }
            data_vec[reg_++] = data;
        } else {
            if (DEBUG_) {
                std::cout << "TwoWire::write(): invalid mode " << static_cast<uint32_t>(mode_) << "\n";
            }
            return 0;
        }

        return 1;
    }

    uint8_t requestFrom(int address, int quantity, [[maybe_unused]] uint8_t sendStop = 1) {
        if (address >= 8 && address < 120) {
            addr_ = address;
            mode_ = 3;
            available_ = quantity;
            if (DEBUG_) {
                std::cout << "TwoWire::requestFrom(): address 0x" << std::hex << static_cast<int32_t>(address) << std::dec << " quantity=" << available_
                          << "\n";
            }
        } else {
            if (DEBUG_) {
                std::cout << "TwoWire::requestFrom(): invalid address 0x" << std::hex << static_cast<int32_t>(address) << std::dec << "\n";
            }
        }

        return available_;
    }

    virtual int available() {
        return available_;
    }

    virtual int read() {
        if (mode_ == 3) {
            auto& data_vec { data_[addr_] };
            if (data_vec.size() < 65'535) {
                data_vec.resize(65'535, 0);
                if (DEBUG_) {
                    std::cout << "TwoWire::read(): vector resized to 65535.\n";
                }
            }
            if (DEBUG_) {
                std::cout << "TwoWire::read(): [0x" << std::hex << static_cast<uint32_t>(reg_) << "] = 0x" << static_cast<uint32_t>(data_vec[reg_]) << std::dec
                          << "\n";
            }
            return data_vec[reg_++];
        }

        if (DEBUG_) {
            std::cout << "TwoWire::read(): invalid mode " << static_cast<uint32_t>(mode_) << "\n";
        }
        return -1;
    }

    size_t readBytes(uint8_t* buffer, size_t length) {
        if (mode_ == 3) {
            size_t n { length };
            if (static_cast<int>(n) > available_) {
                n = std::max<size_t>(0, available_);
            }

            auto& data_vec { data_[addr_] };
            if (data_vec.size() < 65'535) {
                data_vec.resize(65'535, 0);
                if (DEBUG_) {
                    std::cout << "TwoWire::readBytes(): vector resized to 65535.\n";
                }
            }
            for (size_t i {}; i < n; ++i) {
                buffer[i] = data_vec[reg_++];
            }

            return n;
        }

        return 0;
    }

    size_t write(const uint8_t* buffer, size_t length) {
        if (length == 1) {
            return write(*buffer);
        } else if (length == 2) {
            auto res { write(buffer[0]) };
            res += write(buffer[1]);
            return res;
        }

        if (mode_ == 3) {
            auto& data_vec { data_[addr_] };
            if (data_vec.size() < 65'535) {
                data_vec.resize(65'535, 0);
                if (DEBUG_) {
                    std::cout << "TwoWire::write(): vector resized to 65535.\n";
                }
            }
            for (size_t i {}; i < length; ++i) {
                data_vec[reg_++] = buffer[i];
            }

            return length;
        }

        return 0;
    }

    void set_addr_width(const uint8_t addr, const uint8_t width) {
        if (addr >= 8 && addr < 120) {
            addr_width_[addr] = width;
        }
    }
};

extern TwoWire Wire;
extern TwoWire Wire1;
extern TwoWire Wire2;
extern TwoWire Wire3;

namespace arduino {
extern KINETIS_I2C_t i2c_dummy;
}

#define KINETIS_I2C0 (*(KINETIS_I2C_t*) &arduino::i2c_dummy)
#define KINETIS_I2C1 (*(KINETIS_I2C_t*) &arduino::i2c_dummy)
#define KINETIS_I2C2 (*(KINETIS_I2C_t*) &arduino::i2c_dummy)
#define KINETIS_I2C3 (*(KINETIS_I2C_t*) &arduino::i2c_dummy)

/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    I2Cdev.cpp
 * @brief   I2Cdev device library wrapper for ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    04.04.2021
 */

#include "I2Cdev.h"
#include "arduino_freertos.h"
#include "../../../src/i2c_service.h"


I2C_Service* I2Cdev::i2c_ {};

I2Cdev::I2Cdev() = default;

bool I2Cdev::init(const uint8_t bus_id, const uint32_t freq) {
    if (i2c_ && i2c_->get_bus() == bus_id && i2c_->get_freq() == freq) {
        return true;
    }

    delete i2c_;
    i2c_ = new I2C_Service { bus_id, freq, 18, 19 };

    return i2c_ != nullptr;
}

int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data, uint16_t) {
    uint8_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) != I2C_Service::I2C_Error::SUCCESS) {
        return 0;
    }
    *data = tmp & (1 << bitNum);
    return 1;
}

int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t* data, uint16_t) {
    uint16_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) != I2C_Service::I2C_Error::SUCCESS) {
        return 0;
    }
    *data = tmp & (1 << bitNum);
    return 1;
}

int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data, uint16_t) {
    uint8_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) == I2C_Service::I2C_Error::SUCCESS) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        tmp &= mask;
        tmp >>= (bitStart - length + 1);
        *data = tmp;
        return 1;
    }
    return 0;
}

int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t* data, uint16_t) {
    uint16_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) == I2C_Service::I2C_Error::SUCCESS) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        tmp &= mask;
        tmp >>= (bitStart - length + 1);
        *data = tmp;
        return 1;
    }
    return 0;
}

int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t) {
    uint8_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) != I2C_Service::I2C_Error::SUCCESS) {
        return 0;
    }
    *data = tmp;
    return 1;
}

int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t* data, uint16_t) {
    uint16_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) != I2C_Service::I2C_Error::SUCCESS) {
        return 0;
    }
    *data = tmp;
    return 1;
}

int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data, uint16_t) {
    configASSERT(length != 3);

    return i2c_->read_bytes(devAddr, regAddr, data, length) == I2C_Service::I2C_Error::SUCCESS;
}

int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data, uint16_t) {
    uint8_t intermediate[length * 2];
    if (readBytes(devAddr, regAddr, length * 2, intermediate)) {
        for (uint8_t i {}; i < length; ++i) {
            data[i] = (intermediate[2 * i] << 8) | intermediate[2 * i + 1];
        }
        return length;
    }
    return -1;
}

bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    return i2c_->set_bit(devAddr, regAddr, bitNum, data) == I2C_Service::I2C_Error::SUCCESS;
}

bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t tmp;
    if (readWord(devAddr, regAddr, &tmp) != 1) {
        return false;
    }
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, tmp);
}

bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) != I2C_Service::I2C_Error::SUCCESS) {
        return false;
    }
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    return i2c_->write_reg(devAddr, regAddr, tmp) == I2C_Service::I2C_Error::SUCCESS;
}

bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    uint16_t tmp;
    if (i2c_->read_reg(devAddr, regAddr, tmp) != I2C_Service::I2C_Error::SUCCESS) {
        return false;
    }
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing word
    tmp |= data; // combine data with existing word
    return i2c_->write_reg(devAddr, regAddr, tmp) == I2C_Service::I2C_Error::SUCCESS;
}

bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return i2c_->write_reg(devAddr, regAddr, data) == I2C_Service::I2C_Error::SUCCESS;
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return i2c_->write_reg(devAddr, regAddr, data) == I2C_Service::I2C_Error::SUCCESS;
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    configASSERT(length != 3);
    return i2c_->write_bytes(devAddr, regAddr, data, length) == I2C_Service::I2C_Error::SUCCESS;
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    uint8_t intermediate[length * 2];
    for (uint8_t i {}; i < length; i += 2) {
        intermediate[i] = data[i] >> 8;
        intermediate[i + 1] = static_cast<uint8_t>(data[i]);
    }

    return writeBytes(devAddr, regAddr, length * 2, intermediate);
}

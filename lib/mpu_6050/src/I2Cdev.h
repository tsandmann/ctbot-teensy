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
 * @file    I2Cdev.h
 * @brief   I2Cdev device library wrapper for ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    04.04.2021
 */

#pragma once

#include <cstdint>

class I2C_Service;

class I2Cdev {
    static I2C_Service* i2c_;

public:
    I2Cdev() = default;

    static bool init(const uint8_t bus_id, const uint32_t freq);

    static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data, uint16_t timeout = 0);
    static int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t* data, uint16_t timeout = 0);
    static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data, uint16_t timeout = 0);
    static int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t* data, uint16_t timeout = 0);
    static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t timeout = 0);
    static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t* data, uint16_t timeout = 0);
    static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data, uint16_t timeout = 0);
    static int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data, uint16_t timeout = 0);

    static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    static bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
    static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    static bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
    static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
    static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
    static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data);
};

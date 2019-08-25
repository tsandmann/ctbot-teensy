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

#define KINETIS_I2C0 (*(KINETIS_I2C_t*) 0x40066000)
#define KINETIS_I2C1 (*(KINETIS_I2C_t*) 0x40067000)
#define KINETIS_I2C2 (*(KINETIS_I2C_t*) 0x400E6000)
#define KINETIS_I2C3 (*(KINETIS_I2C_t*) 0x400E7000)


class TwoWire {
public:
    void begin() {}
    void setSDA(uint8_t pin) {
        (void) pin;
    }
    void setSCL(uint8_t pin) {
        (void) pin;
    }
    void setClock(uint32_t frequency) {
        (void) frequency;
    }
    void beginTransmission(uint8_t address) {
        (void) address;
    }
    uint8_t endTransmission() {
        return 0;
    }
    virtual size_t write(uint8_t data) {
        (void) data;
        return 0;
    }
    uint8_t requestFrom(int address, int quantity) {
        (void) address;
        (void) quantity;
        return 0;
    }
    virtual int available() {
        return 0;
    }
    virtual int read() {
        return -1;
    }
};

extern TwoWire Wire;
extern TwoWire Wire1;
extern TwoWire Wire2;

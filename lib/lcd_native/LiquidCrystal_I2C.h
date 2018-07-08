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
 * @file    LiquidCrystal_I2C.h
 * @brief   Wrapper aroung Arduino LiquidCrystal_I2C library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#ifndef LiquidCrystal_I2C_NATIVE_H
#define LiquidCrystal_I2C_NATIVE_H

#include <cstdint>
#include <cstddef>
#include <cstdio>


class LiquidCrystal_I2C {
protected:
    FILE* p_lcd_out;

public:
    typedef enum { POSITIVE, NEGATIVE } t_backlighPol;

    LiquidCrystal_I2C(uint8_t i2c_port, uint8_t lcd_Addr, uint8_t En, uint8_t Rw, uint8_t Rs, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

    virtual bool begin(uint8_t cols, uint8_t rows);

    virtual void send(uint8_t value, uint8_t mode);

    void setBacklightPin(uint8_t value, t_backlighPol pol);

    void setBacklight(uint8_t value);

    void clear();

    void setCursor(uint8_t col, uint8_t row);

    size_t print(char c);

    size_t print(const char s[]);

    void set_output(FILE* fp) {
        p_lcd_out = fp;
    }
};

#endif /* LiquidCrystal_I2C_NATIVE_H */

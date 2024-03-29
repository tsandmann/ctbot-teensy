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
 * @file    LiquidCrystal_I2C.cpp
 * @brief   Wrapper aroung Arduino LiquidCrystal_I2C library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#include "LiquidCrystal_I2C.h"

#include <cstring>


LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t)
    : p_lcd_out {}, cursor_r_ {}, cursor_c_ {} {
    for (auto& e : data_) {
        e.resize(20, ' ');
    }
}

LiquidCrystal_I2C::~LiquidCrystal_I2C() = default;

bool LiquidCrystal_I2C::begin(uint8_t, uint8_t) {
    return true;
}

void LiquidCrystal_I2C::setBacklight(uint8_t) {}

void LiquidCrystal_I2C::setBacklightPin(uint8_t, LiquidCrystal_I2C::t_backlighPol) {}

void LiquidCrystal_I2C::send(uint8_t, uint8_t) {}

void LiquidCrystal_I2C::clear() {
    if (p_lcd_out) {
        fprintf(p_lcd_out, "\033[2J");
        fflush(p_lcd_out);
    }
    setCursor(0, 0);
}

void LiquidCrystal_I2C::setCursor(uint8_t col, uint8_t row) {
    if (p_lcd_out) {
        fprintf(p_lcd_out, "\033[%d;%dH", row, col);
        fflush(p_lcd_out);
    }

    if (row < 4) {
        cursor_r_ = row;
    }
    if (col < 20) {
        cursor_c_ = col;
    }
}

size_t LiquidCrystal_I2C::write(uint8_t b) {
    if (p_lcd_out) {
        fputc(b, p_lcd_out);
    }

    data_[cursor_r_][cursor_c_] = b;
    cursor_c_ = (cursor_c_ + 1) % 20;

    return 1;
}

size_t LiquidCrystal_I2C::write(const uint8_t* buf, size_t size) {
    if (p_lcd_out) {
        for (size_t i {}; i < size; ++i) {
            fputc(*buf++, p_lcd_out);
        }
    }

    size_t i, n {};
    for (i = cursor_c_; i < cursor_c_ + size && i < 20; ++i) {
        char tmp { static_cast<char>(*buf++) };
        if (tmp < 0x20) {
            tmp = ' ';
        } else if (tmp > 0x7e) {
            tmp = '#';
        }
        data_[cursor_r_][i] = tmp;
        ++n;
    }
    cursor_c_ = i % 20;

    return n;
}

size_t LiquidCrystal_I2C::write(const char* s) {
    return write(s, std::strlen(s));
}

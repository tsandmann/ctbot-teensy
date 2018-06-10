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
#include <iostream>


LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw, uint8_t Rs, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
}

bool LiquidCrystal_I2C::begin(uint8_t cols, uint8_t lines) {
   return true;
}

void LiquidCrystal_I2C::setBacklight(uint8_t value) {
}

void LiquidCrystal_I2C::setBacklightPin(uint8_t value, LiquidCrystal_I2C::t_backlighPol pol) {
}

void LiquidCrystal_I2C::send(uint8_t value, uint8_t mode) {
}

void LiquidCrystal_I2C::clear() {
}

void LiquidCrystal_I2C::setCursor(uint8_t col, uint8_t row) {
}

size_t LiquidCrystal_I2C::print(char c) {
    // std::cout << c;
    return 1;
}

size_t LiquidCrystal_I2C::print(const char s[]) {
    // std::cout << s << "\n";
    return std::strlen(s);
}

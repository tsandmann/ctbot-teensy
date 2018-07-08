/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    display.cpp
 * @brief   LC display driver for devices with Hitachi HD44780 and PCF8574 i2c i/o expander
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "display.h"

#include <LiquidCrystal_I2C.h>
#include <arduino_fixed.h>


namespace ctbot {

Display::Display() : p_impl_ { new LiquidCrystal_I2C(CtBotConfig::I2C_FOR_LCD, 0x3f, 2, 1, 0, 4, 5, 6, 7) } {
    arduino::Wire2.setSDA(CtBotConfig::I2C2_PIN_SDA);
    arduino::Wire2.setSCL(CtBotConfig::I2C2_PIN_SCL);

    p_impl_->begin(LINE_LENGTH, 4);
    p_impl_->setBacklightPin(3, LiquidCrystal_I2C::POSITIVE);
    p_impl_->setBacklight(false);
    clear();
}

void Display::clear() const {
    p_impl_->clear();
}

void Display::set_cursor(const uint8_t row, const uint8_t column) const {
    const uint8_t c { static_cast<uint8_t>(column - 1) };
    if (c >= LINE_LENGTH) {
        return;
    }
    p_impl_->setCursor(c, row - 1);
}

void Display::set_backlight(const bool status) const {
    p_impl_->setBacklight(status);
}

uint8_t Display::print(const char c) const {
    return p_impl_->print(c);
}

uint8_t Display::print(const std::string& str) const {
    return p_impl_->print(str.c_str()); // baeh
}

uint8_t Display::printf(const char* format, ...) {
    va_list args;
    va_start(args, format);

    /* read C-string and parse format */
    uint8_t len(vsnprintf(buffer_, sizeof(buffer_), reinterpret_cast<const char*>(format), args));
    va_end(args);

    /* truncate to line length */
    if (len > LINE_LENGTH) {
        len = LINE_LENGTH;
    }

    /* send characters to display */
    const char* ptr(buffer_);
    for (uint8_t i { 0U }; i < len; ++i) {
        p_impl_->print(*ptr++);
    }

    return len;
}

} // namespace ctbot

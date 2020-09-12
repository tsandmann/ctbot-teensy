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
 * @file    lc_display.cpp
 * @brief   LC display driver for devices with Hitachi HD44780 and PCF8574 i2c i/o expander
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "lc_display.h"
#include "scheduler.h"

#include "LiquidCrystal_I2C.h"
#include "arduino_freertos.h"

#include <cstdarg>


namespace ctbot {

LCDisplay::LCDisplay() : p_impl_ { new LiquidCrystal_I2C { CtBotConfig::LCD_I2C_BUS, 0x3f, 2, 1, 0, 4, 5, 6, 7 } } {
    Scheduler::enter_critical_section();
    // arduino::Wire2.setSDA(CtBotConfig::I2C2_PIN_SDA);
    // arduino::Wire2.setSCL(CtBotConfig::I2C2_PIN_SCL);

    p_impl_->begin(LINE_LENGTH, 4);
    p_impl_->setBacklightPin(3, LiquidCrystal_I2C::POSITIVE);
    p_impl_->setBacklight(false);
    Scheduler::exit_critical_section();

    clear();
}

LCDisplay::~LCDisplay() {
    delete p_impl_;
}

void LCDisplay::clear() const {
    Scheduler::enter_critical_section();
    p_impl_->clear();
    Scheduler::exit_critical_section();
}

void LCDisplay::set_cursor(const uint8_t row, const uint8_t column) const {
    const uint8_t c { static_cast<uint8_t>(column - 1) };
    if (c >= LINE_LENGTH) {
        return;
    }
    Scheduler::enter_critical_section();
    p_impl_->setCursor(c, row - 1);
    Scheduler::exit_critical_section();
}

void LCDisplay::set_backlight(const bool status) const {
    Scheduler::enter_critical_section();
    p_impl_->setBacklight(status);
    Scheduler::exit_critical_section();
}

uint8_t LCDisplay::print(const char c) const {
    Scheduler::enter_critical_section();
    const uint8_t ret { static_cast<uint8_t>(p_impl_->write(c)) };
    Scheduler::exit_critical_section();
    return ret;
}

uint8_t LCDisplay::print(const std::string_view& str) const {
    Scheduler::enter_critical_section();
    const uint8_t ret { static_cast<uint8_t>(p_impl_->write(str.data(), str.size())) };
    Scheduler::exit_critical_section();
    return ret;
}

uint8_t LCDisplay::printf(const char* format, ...) {
    va_list args;
    va_start(args, format);

    /* read C-string and parse format */
    auto len { std::vsnprintf(buffer_, sizeof(buffer_), reinterpret_cast<const char*>(format), args) };
    va_end(args);

    /* truncate to line length */
    if (len > LINE_LENGTH) {
        len = LINE_LENGTH;
    }

    /* send characters to display */
    const char* ptr(buffer_);
    Scheduler::enter_critical_section();
    for (uint8_t i {}; i < len; ++i) {
        p_impl_->write(*ptr++);
    }
    Scheduler::exit_critical_section();

    return static_cast<uint8_t>(len);
}

void LCDisplay::set_output(const std::string_view& out) {
    if (out == "stdout") {
        p_impl_->set_output(stdout);
    } else {
        p_impl_->set_output(nullptr);
    }
}

} // namespace ctbot

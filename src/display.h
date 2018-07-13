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
 * @file    display.h
 * @brief   LC display driver for devices with Hitachi HD44780 and PCF8574 i2c i/o expander
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#ifndef SRC_DISPLAY_H_
#define SRC_DISPLAY_H_

#include "ctbot_config.h"

#include <cstdint>
#include <string>


class LiquidCrystal_I2C;

namespace ctbot {

/**
 * @brief LC Display driver implementation for devices with Hitachi HD44780 and PCF8574 i2c i/o expander
 */
class Display {
protected:
    static constexpr uint8_t LINE_LENGTH = 20; /**< Size of display (length of one line) */
    LiquidCrystal_I2C* p_impl_;
    char buffer_[LINE_LENGTH + 1];

public:
    Display();

    /**
     * @brief Clear entire display, set cursor to home position
     */
    void clear() const;

    /**
     * @brief Set the cursor to a position
     * @param[in] row: New row of cursor [1; 20]
     * @param[in] column: New column of cursor [1; 4]
     */
    void set_cursor(const uint8_t row, const uint8_t column) const;

    /**
     * @brief Set the display backlight on or off
     * @param[in] status: true or false to switch backlight on or off
     */
    void set_backlight(const bool status) const;

    /**
     * @brief Write a char to the display, starting at the current position
     * @param[in] c: Character to write
     * @return Number of written chars (1)
     */
    uint8_t print(const char c) const;

    /**
     * @brief Write a string to the display, starting at the current position
     * @param[in] str: Reference to the string
     * @return Number of written chars
     */
    uint8_t print(const std::string& str) const;

    /**
     * @brief Write a formatted C-string to the display, starting at the current position
     * @param[in] format: printf()-like format string
     * @param[in] ...: Variadic argument list as for printf()
     * @return Number of written chars
     */
    uint8_t printf(const char* format, ...);
};

} // namespace ctbot

#endif /* SRC_DISPLAY_H_ */

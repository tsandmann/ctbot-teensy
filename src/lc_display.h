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
 * @file    lc_display.h
 * @brief   LC display driver for devices with Hitachi HD44780 and PCF8574 i2c i/o expander
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "ctbot_config.h"

#include <cstdint>
#include <string>
#include <string_view>


class LiquidCrystal_I2C;

namespace ctbot {

/**
 * @brief LC Display driver implementation for devices with Hitachi HD44780 and PCF8574 i2c i/o expander
 *
 * @startuml{Display.png}
 *  !include display.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class LCDisplay {
protected:
    static constexpr uint8_t LINE_LENGTH { 20 }; /**< Size of display (length of one line) */
    LiquidCrystal_I2C* p_impl_;
    char buffer_[LINE_LENGTH + 1];

public:
    FLASHMEM LCDisplay();

    FLASHMEM ~LCDisplay();

    /**
     * @brief Clear entire display, set cursor to home position
     */
    FLASHMEM void clear() const;

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
    FLASHMEM uint8_t print(const char c) const;

    /**
     * @brief Write a string to the display, starting at the current position
     * @param[in] str: Reference to the string as string_view
     * @return Number of written chars
     */
    FLASHMEM uint8_t print(const std::string_view& str) const;

    /**
     * @brief Write a formatted C-string to the display, starting at the current position
     * @param[in] format: printf()-like format string
     * @param[in] ...: Variadic argument list as for printf()
     * @return Number of written chars
     */
    FLASHMEM uint8_t printf(const char* format, ...);

    /**
     * @brief Enable display output to a file
     * @param[in] out: Output file name
     * @note Currently only stdout is implemented
     */
    FLASHMEM void set_output(const std::string_view& out);

    FLASHMEM auto get_impl() {
        return p_impl_;
    }
};

} // namespace ctbot

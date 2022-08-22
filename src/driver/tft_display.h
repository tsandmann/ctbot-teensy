/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
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
 * @file    tft_display.h
 * @brief   TFT display driver for devices with ILI9341 controller
 * @author  Timo Sandmann
 * @date    10.04.2019
 */

#pragma once

#include "ctbot_config.h"
#include "logger.h"

#include <cstdint>
#include <memory>
#include <string>


class Adafruit_ILI9341;
class XPT2046_Touchscreen;
class Adafruit_GFX;

namespace ctbot {

class TFTColorHelper {
public:
    /**
     * @brief Given 8-bit red, green and blue values, return a 'packed' 16-bit color value
     * in '565' RGB format (5 bits red, 6 bits green, 5 bits blue)
     * @param[in] red: 8-bit red brightnesss (0 = off, 255 = max)
     * @param[in] green: 8-bit green brightnesss (0 = off, 255 = max)
     * @param[in] blue: 8-bit blue brightnesss (0 = off, 255 = max)
     * @return 'Packed' 16-bit color value (565 format)
     */
    static constexpr uint16_t color565(const uint8_t red, const uint8_t green, const uint8_t blue) {
        return ((red & 0xf8) << 8) | ((green & 0xfc) << 3) | (blue >> 3);
    }
};

class TFTColors : public TFTColorHelper {
public:
    static constexpr uint16_t BLACK { color565(0, 0, 0) };
    static constexpr uint16_t NAVY { color565(0, 0, 123) };
    static constexpr uint16_t DARKGREEN { color565(0, 125, 0) };
    static constexpr uint16_t DARKCYAN { color565(0, 125, 123) };
    static constexpr uint16_t MAROON { color565(123, 0, 0) };
    static constexpr uint16_t PURPLE { color565(123, 0, 123) };
    static constexpr uint16_t OLIVE { color565(123, 125, 0) };
    static constexpr uint16_t LIGHTGREY { color565(198, 195, 198) };
    static constexpr uint16_t DARKGREY { color565(123, 125, 123) };
    static constexpr uint16_t BLUE { color565(0, 0, 255) };
    static constexpr uint16_t GREEN { color565(0, 255, 0) };
    static constexpr uint16_t CYAN { color565(0, 255, 255) };
    static constexpr uint16_t RED { color565(255, 0, 0) };
    static constexpr uint16_t MAGENTA { color565(255, 0, 255) };
    static constexpr uint16_t YELLOW { color565(255, 255, 0) };
    static constexpr uint16_t WHITE { color565(255, 255, 255) };
    static constexpr uint16_t ORANGE { color565(255, 165, 0) };
    static constexpr uint16_t GREENYELLOW { color565(173, 255, 41) };
    static constexpr uint16_t PINK { color565(255, 130, 198) };
};

class TFTDisplay {
protected:
    Adafruit_ILI9341* p_display_;
    XPT2046_Touchscreen* p_touch_;
    uint16_t width_;
    uint16_t height_;

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] p_str: Message as pointer to std::string
     * @return Number of characters written
     */
    FLASHMEM uint8_t print(const std::string* p_str, bool clear = false) const;

public:
    FLASHMEM TFTDisplay();

    FLASHMEM ~TFTDisplay();

    auto get_width() const {
        return width_;
    }

    auto get_height() const {
        return height_;
    }

    /**
     * @brief Clear entire display, set cursor to home position
     */
    void clear() const;

    void clear(const uint8_t row) const;

    /**
     * @brief Set the cursor to a position
     * @param[in] row: New row of cursor [1; 20]
     * @param[in] column: New column of cursor [1; 4]
     */
    void set_cursor_line(const uint8_t row, const uint8_t column) const;

    void set_cursor(const int16_t x, const int16_t y) const;

    void set_text_size(const uint8_t size) const;

    void set_text_color(const uint16_t color) const;

    /**
     * @brief Set the display backlight
     * @param[in] brightness: new brightness in percentage
     */
    void set_backlight(const float brightness) const;

    /**
     * @brief Write a char to the display, starting at the current position
     * @param[in] c: Character to write
     * @return Number of written chars (1)
     */
    FLASHMEM uint8_t print(const char c, bool clear = false) const;

    /**
     * @brief Write a string to the display, starting at the current position
     * @param[in] str: Reference to the string
     * @return Number of written chars
     */
    FLASHMEM uint8_t print(const std::string& str, bool clear = false) const;

    /**
     * @brief Write an formatted string (like std::printf) out to display
     * @tparam Args: Values to print
     * @param[in] format: Format string
     * @return Number of characters written
     */
    template <typename... Args>
    FLASHMEM uint8_t printf(const char* format, const Args&... args) const {
        return print(LoggerTarget::string_format(format, args...));
    }

    int16_t get_cursor_x() const;

    int16_t get_cursor_y() const;

    FLASHMEM void get_text_bounds(const std::string& str, const int16_t x, const int16_t y, int16_t* p_x, int16_t* p_y, uint16_t* p_w, uint16_t* p_h) const;

    FLASHMEM void fill_screen(const uint16_t color) const;

    FLASHMEM void fill_rect(const int16_t x, const int16_t y, const int16_t w, const int16_t h, const uint16_t color) const;

    FLASHMEM void draw_line(const int16_t x0, const int16_t y0, const int16_t x1, const int16_t y1, const uint16_t color) const;

    FLASHMEM void draw_rect(const int16_t x, const int16_t y, const int16_t w, const int16_t h, const uint16_t color) const;

    FLASHMEM void draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) const;

    FLASHMEM void draw_circle(const int16_t x, const int16_t y, const int16_t r, const uint16_t color) const;

    bool touched() const;

    FLASHMEM bool get_touch_point(int16_t& x, int16_t& y, int16_t& z) const;

    Adafruit_GFX* get_context() const {
        return reinterpret_cast<Adafruit_GFX*>(p_display_);
    }
};

} // namespace ctbot

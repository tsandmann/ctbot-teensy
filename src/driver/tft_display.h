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
 * @brief   TFT display driver
 * @author  Timo Sandmann
 * @date    10.04.2019
 */

#pragma once

#include "ctbot_config.h"
#include "logger.h"
#include "leds_i2c.h"

#define FOONATHAN_HAS_EXCEPTION_SUPPORT 0
#include "memory_pool.hpp"
#include "namespace_alias.hpp"
#include "static_allocator.hpp"
#undef FOONATHAN_HAS_EXCEPTION_SUPPORT

#include "avr/pgmspace.h"

#include <cstdint>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <array>

#ifndef EXTMEM
#define EXTMEM
#endif


class TFT_SPI;
class Adafruit_GFX;
class Adafruit_GFX_Button;
class GFXcanvas16;
class XPT2046_Touchscreen;
class TS_Point;

namespace ctbot {

class TFTColorHelper {
public:
    /**
     * @brief Given 8-bit red, green and blue values, return a 'packed' 16-bit color value
     *        in '565' RGB format (5 bits red, 6 bits green, 5 bits blue)
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
    static constexpr bool DEBUG_ { false };

protected:
    static constexpr uint32_t FRAMES_PER_SEC_ { 10 };
    static constexpr int16_t WIDTH_ { CtBotConfig::TFT_CONTROLLER_TYPE == 9486 ? 480 : 320 };
    static constexpr int16_t HEIGHT_ { CtBotConfig::TFT_CONTROLLER_TYPE == 9486 ? 320 : 240 };
    static constexpr uint16_t TOUCH_THRESHOLD_ { 800 };
    static constexpr size_t FB_CHUNK_SIZE_ { WIDTH_ };

    using static_pool_t = memory::memory_pool<memory::array_pool, memory::static_allocator>;
    EXTMEM static memory::static_allocator_storage<WIDTH_ * HEIGHT_ * sizeof(uint16_t) + 16> framebuffer_pool_;

    static_pool_t mem_pool_ { FB_CHUNK_SIZE_, sizeof(framebuffer_pool_), framebuffer_pool_ };
    std::array<uint16_t, WIDTH_ * HEIGHT_>* framebuffer_mem_;
    TFT_SPI* p_display_;
    GFXcanvas16* p_framebuffer_;
    XPT2046_Touchscreen* p_touch_;
    TaskHandle_t task_;
    std::atomic<bool> service_running_;
    mutable std::atomic<bool> updated_;
    mutable std::mutex fb_mutex_;
    LedsI2cEna<>* p_backl_pwm_;
    uint32_t touch_counter_;
    TS_Point* p_touch_point_;
    mutable std::mutex touch_mutex_;

    decltype(framebuffer_mem_) init_memory();

    void run();

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] p_str: Message as pointer to std::string
     * @return Number of characters written
     */
    FLASHMEM uint8_t print(const std::string* p_str, bool clear = false) const;

public:
    FLASHMEM TFTDisplay(LedsI2cEna<>& backl_pwm);

    FLASHMEM ~TFTDisplay();

    auto get_width() const {
        return WIDTH_;
    }

    auto get_height() const {
        return HEIGHT_;
    }

    /**
     * @brief Clear entire display, set cursor to home position
     */
    void clear() const;

    void clear(uint8_t row) const;

    void flush() const;

    /**
     * @brief Set the cursor to a position
     * @param[in] row: New row of cursor [1; 20]
     * @param[in] column: New column of cursor [1; 4]
     */
    void set_cursor_line(uint8_t row, uint8_t column) const;

    void set_cursor(int16_t x, int16_t y) const;

    void set_text_size(uint8_t size) const;

    void set_text_color(uint16_t color) const;

    void set_text_wrap(bool wrap) const;

    /**
     * @brief Set the display backlight
     * @param[in] brightness: new brightness in percentage
     */
    void set_backlight(float brightness) const;

    /**
     * @brief Write a char to the display, starting at the current position
     * @param[in] c: Character to write
     * @return Number of written chars (1)
     */
    FLASHMEM uint8_t print(char c, bool clear = false) const;

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
        if (const auto str { LoggerTarget::string_format(format, args...) }; str.has_value()) {
            return print(*str);
        } else {
            return 0;
        }
    }

    int16_t get_cursor_x() const;

    int16_t get_cursor_y() const;

    void get_text_bounds(const std::string& str, int16_t x, int16_t y, int16_t* p_x, int16_t* p_y, uint16_t* p_w, uint16_t* p_h) const;

    void fill_screen(uint16_t color) const;

    void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) const;

    void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) const;

    void draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) const;

    void draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) const;

    void fill_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) const;

    void draw_circle(int16_t x, int16_t y, int16_t r, uint16_t color) const;

    void fill_circle(int16_t x, int16_t y, int16_t r, uint16_t color) const;

    void draw_button(Adafruit_GFX_Button* button, bool invert = false) const;

    auto get_touch_counter() const {
        std::lock_guard<std::mutex> lock { touch_mutex_ };
        return touch_counter_;
    }

    bool get_touch_point(int16_t& x, int16_t& y, int16_t& z) const;

    Adafruit_GFX* get_context() const {
        return reinterpret_cast<Adafruit_GFX*>(p_framebuffer_);
    }
};

} // namespace ctbot

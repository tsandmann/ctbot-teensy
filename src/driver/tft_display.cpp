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
 * @file    tft_display.cpp
 * @brief   TFT display driver for devices with ILI9341 controller
 * @author  Timo Sandmann
 * @date    10.04.2019
 */

#include "tft_display.h"

#include "scheduler.h"

#include "Adafruit_ILI9341.h"
#include "XPT2046_Touchscreen.h"
#include "arduino_freertos.h"


namespace ctbot {
TFTDisplay::TFTDisplay()
    : p_display_ { new Adafruit_ILI9341(CtBotConfig::TFT_SPI == 0 ? &SPI :
            CtBotConfig::TFT_SPI == 1                             ? &SPI1 :
                                                                    &SPI2,
        CtBotConfig::TFT_CS_PIN, CtBotConfig::TFT_DC_PIN, -1) },
      p_touch_ { new XPT2046_Touchscreen { CtBotConfig::TFT_TOUCH_CS_PIN,
          CtBotConfig::TFT_SPI == 0     ? &SPI :
              CtBotConfig::TFT_SPI == 1 ? &SPI1 :
                                          &SPI2 } } {
    if (!p_display_) {
        return;
    }

    if (!p_touch_) {
        delete p_display_;
        return;
    }

    Scheduler::enter_critical_section();
    arduino::pinMode(CtBotConfig::TFT_BACKLIGHT_PIN, arduino::OUTPUT);
    arduino::analogWriteResolution(16);
    arduino::analogWriteFrequency(CtBotConfig::TFT_BACKLIGHT_PIN, 915.527f);
    set_backlight(CtBotConfig::TFT_BACKLIGHT_LEVEL);

    if (CtBotConfig::TFT_SPI == 1) {
        arduino::SPI.setMOSI(CtBotConfig::SPI1_PIN_MOSI);
        arduino::SPI.setMISO(CtBotConfig::SPI1_PIN_MISO);
        arduino::SPI.setSCK(CtBotConfig::SPI1_PIN_SCK);
        arduino::SPI.setCS(CtBotConfig::TFT_CS_PIN);
    } else if (CtBotConfig::TFT_SPI == 2) {
        arduino::SPI1.setMOSI(CtBotConfig::SPI2_PIN_MOSI);
        arduino::SPI1.setMISO(CtBotConfig::SPI2_PIN_MISO);
        arduino::SPI1.setSCK(CtBotConfig::SPI2_PIN_SCK);
        arduino::SPI1.setCS(CtBotConfig::TFT_CS_PIN);
    }

    p_display_->begin(CtBotConfig::TFT_SPI_FREQUENCY);
    p_touch_->begin();
    Scheduler::exit_critical_section();

    p_display_->setRotation(CtBotConfig::TFT_ROTATION);
    p_display_->setTextWrap(false);
    width_ = p_display_->width();
    height_ = p_display_->height();
    clear();

    p_touch_->setRotation(CtBotConfig::TFT_TOUCH_ROTATION);
}

TFTDisplay::~TFTDisplay() {
    delete p_touch_;
    delete p_display_;
}

void TFTDisplay::clear() const {
    p_display_->fillScreen(TFTColors::BLACK);
}

void TFTDisplay::clear(const uint8_t row) const {
    p_display_->fillRect(0, height_ * row / 10, width_, height_ / 10, TFTColors::BLACK);
}

void TFTDisplay::set_cursor_line(const uint8_t row, const uint8_t column) const {
    p_display_->setCursor(width_ * column / 28, height_ * row / 10);
}

void TFTDisplay::set_cursor(const int16_t x, const int16_t y) const {
    p_display_->setCursor(x, y);
}

void TFTDisplay::set_text_size(const uint8_t size) const {
    p_display_->setTextSize(size);
}

void TFTDisplay::set_text_color(const uint16_t color) const {
    p_display_->setTextColor(color);
}

void TFTDisplay::set_backlight(const float brightness) const {
    arduino::analogWrite(CtBotConfig::TFT_BACKLIGHT_PIN, static_cast<int>(65535.f - (65535.f * brightness / 100.f)));
}

uint8_t TFTDisplay::print(const char c, bool clear) const {
    if (clear) {
        int16_t x, y;
        uint16_t w {}, h {};
        char tmp[2] { c, '\0' };
        p_display_->getTextBounds(tmp, 0, 0, &x, &y, &w, &h);
        fill_rect(get_cursor_x(), get_cursor_y(), w, h, TFTColors::BLACK);
    }
    return p_display_->write(static_cast<uint8_t>(c));
}

uint8_t TFTDisplay::print(const std::string& str, bool clear) const {
    if (clear) {
        int16_t x, y;
        uint16_t w {}, h {};
        p_display_->getTextBounds(str.c_str(), 0, 0, &x, &y, &w, &h);
        fill_rect(get_cursor_x(), get_cursor_y(), w, h, TFTColors::BLACK);
    }
    return p_display_->Print::write(str.data(), str.length());
}

uint8_t TFTDisplay::print(const std::string* p_str, bool clear) const {
    const auto ret { print(*p_str, clear) };
    delete p_str;
    return ret;
}

int16_t TFTDisplay::get_cursor_x() const {
    return p_display_->getCursorX();
}

int16_t TFTDisplay::get_cursor_y() const {
    return p_display_->getCursorY();
}

void TFTDisplay::get_text_bounds(const std::string& str, const int16_t x, const int16_t y, int16_t* p_x, int16_t* p_y, uint16_t* p_w, uint16_t* p_h) const {
    p_display_->getTextBounds(str.c_str(), x, y, p_x, p_y, p_w, p_h);
}

void TFTDisplay::fill_screen(const uint16_t color) const {
    p_display_->fillScreen(color);
}

void TFTDisplay::fill_rect(const int16_t x, const int16_t y, const int16_t w, const int16_t h, const uint16_t color) const {
    p_display_->fillRect(x, y, w, h, color);
}

void TFTDisplay::draw_line(const int16_t x0, const int16_t y0, const int16_t x1, const int16_t y1, const uint16_t color) const {
    p_display_->drawLine(x0, y0, x1, y1, color);
}

void TFTDisplay::draw_rect(const int16_t x, const int16_t y, const int16_t w, const int16_t h, const uint16_t color) const {
    p_display_->drawRect(x, y, w, h, color);
}

void TFTDisplay::draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) const {
    p_display_->drawTriangle(x0, y0, x1, y1, x2, y2, color);
}

void TFTDisplay::draw_circle(const int16_t x, const int16_t y, const int16_t r, const uint16_t color) const {
    p_display_->drawCircle(x, y, r, color);
}

bool TFTDisplay::touched() const {
    return p_touch_->touched();
}

bool TFTDisplay::get_touch_point(int16_t& x, int16_t& y, int16_t& z) const {
    auto point { p_touch_->getPoint() };
    x = point.x;
    y = point.y;
    z = point.z;
    return point.z;
}

} // namespace ctbot

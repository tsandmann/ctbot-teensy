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
 * @brief   TFT display driver
 * @author  Timo Sandmann
 * @date    10.04.2019
 */

#include "tft_display.h"

#include "ili9341.h"
#include "ili9486.h"

#include "ctbot.h"
#include "scheduler.h"
#include "timer.h"

#include "Adafruit_GFX.h"
#include "XPT2046_Touchscreen.h"

#include "arduino_freertos.h"


namespace ctbot {

decltype(TFTDisplay::framebuffer_pool_) TFTDisplay::framebuffer_pool_;

TFTDisplay::TFTDisplay(LedsI2cEna<>& backl_pwm)
    : framebuffer_mem_ { init_memory() }, p_display_ { CtBotConfig::TFT_CONTROLLER_TYPE == 9486 ?
              static_cast<TFT_SPI*>(new ILI9486 {
                  CtBotConfig::TFT_SPI == 1 ? &SPI : (CtBotConfig::TFT_SPI == 2 ? &SPI1 : nullptr), CtBotConfig::TFT_CS_PIN, CtBotConfig::TFT_DC_PIN }) :
              (CtBotConfig::TFT_CONTROLLER_TYPE == 9341 ? static_cast<TFT_SPI*>(new ILI9341 {
                   CtBotConfig::TFT_SPI == 1 ? &SPI : (CtBotConfig::TFT_SPI == 2 ? &SPI1 : nullptr), CtBotConfig::TFT_CS_PIN, CtBotConfig::TFT_DC_PIN, -1 }) :
                                                          nullptr) },
      p_framebuffer_ { new GFXcanvas16 { WIDTH_, HEIGHT_, framebuffer_mem_->data() } },
      p_touch_ { new XPT2046_Touchscreen {
          TOUCH_THRESHOLD_, CtBotConfig::TFT_TOUCH_CS_PIN, CtBotConfig::TFT_SPI == 1 ? &SPI : (CtBotConfig::TFT_SPI == 2 ? &SPI1 : nullptr) } },
      service_running_ {}, updated_ {}, p_backl_pwm_ { &backl_pwm }, touch_counter_ {}, p_touch_point_ { new TS_Point {} } {
    static_assert(CtBotConfig::TFT_CONTROLLER_TYPE == 9486 || CtBotConfig::TFT_CONTROLLER_TYPE == 9341, "Unknown TFT controller selected in CtBotConfig");

    configASSERT(p_touch_point_);

    if (!framebuffer_mem_) {
        delete p_touch_point_;
        return;
    }

    if (!p_display_) {
        delete p_touch_point_;
        return;
    }

    if (!p_framebuffer_) {
        delete p_touch_point_;
        delete p_display_;
        return;
    }

    if (!p_touch_) {
        delete p_touch_point_;
        delete p_framebuffer_;
        delete p_display_;
        return;
    }

    if constexpr (CtBotConfig::MAINBOARD_REVISION == 9'000) {
        arduino::pinMode(CtBotConfig::TFT_BACKLIGHT_PIN, arduino::OUTPUT);
        arduino::analogWriteResolution(16);
        arduino::analogWriteFrequency(CtBotConfig::TFT_BACKLIGHT_PIN, 915.527f);
        set_backlight(CtBotConfig::TFT_BACKLIGHT_LEVEL);
    }

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

    p_touch_->begin();
    p_touch_->setRotation(CtBotConfig::TFT_TOUCH_ROTATION);
    p_display_->begin(CtBotConfig::TFT_SPI_FREQUENCY);
    p_display_->set_rotation(CtBotConfig::TFT_ROTATION);
    p_framebuffer_->setTextWrap(false);

    clear();
    updated_ = true;

    ::xTaskCreate(
        [](void* param) {
            auto p_this { reinterpret_cast<TFTDisplay*>(param) };
            p_this->service_running_ = true;
            p_this->run();
        },
        PSTR("TFT Svc"), 384, this, 2, &task_);
}

TFTDisplay::~TFTDisplay() {
    service_running_ = false;
    auto task_state { eTaskGetState(task_) };
    while (task_state != eDeleted && task_state != eInvalid) {
        ::vTaskDelay(1);
        task_state = eTaskGetState(task_);
    }
    delete p_touch_;
    delete p_framebuffer_;
    delete p_display_;

    mem_pool_.deallocate_array(p_framebuffer_, WIDTH_ * HEIGHT_ * sizeof(uint16_t) / FB_CHUNK_SIZE_);
}

decltype(TFTDisplay::framebuffer_mem_) TFTDisplay::init_memory() {
    framebuffer_mem_ = nullptr;
    auto ptr { mem_pool_.try_allocate_array(WIDTH_ * HEIGHT_ * sizeof(uint16_t) / FB_CHUNK_SIZE_) };
    if (ptr) {
        framebuffer_mem_ = new (ptr) std::array<uint16_t, WIDTH_ * HEIGHT_>;
        static_assert(sizeof(*framebuffer_mem_) == WIDTH_ * HEIGHT_ * sizeof((*framebuffer_mem_)[0]) / FB_CHUNK_SIZE_ * FB_CHUNK_SIZE_);
    }

    return framebuffer_mem_;
}

void TFTDisplay::run() {
    TickType_t xLastWakeTime {};
    while (service_running_) {
        if (updated_) {
            uint32_t start, end;
            {
                std::lock_guard<std::mutex> lock { fb_mutex_ };
                if constexpr (DEBUG_) {
                    start = freertos::get_us();
                }
                p_display_->draw_framebuffer(framebuffer_mem_->data());
                if constexpr (DEBUG_) {
                    end = freertos::get_us();
                }
                updated_ = false;
            }
            if constexpr (DEBUG_) {
                const uint32_t diff { (end - start) / 1'000 };
                CtBot::get_instance().get_comm()->debug_printf<false>(PSTR("TFT update took %u ms, possible FPS: %.2f\r\n"), diff, 1'000.f / diff);
            }
        }

        if (p_touch_->touched()) {
            {
                std::lock_guard<std::mutex> lock { touch_mutex_ };
                *p_touch_point_ = p_touch_->getPoint();
                ++touch_counter_;
            }
            if constexpr (DEBUG_) {
                CtBot::get_instance().get_comm()->debug_printf<false>(
                    PSTR("TFT touched: %d %d %d\r\n"), p_touch_point_->x, p_touch_point_->y, p_touch_point_->z);
            }
        } else {
            std::lock_guard<std::mutex> lock { touch_mutex_ };
            p_touch_point_->z = -1;
        }
        ::xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1'000UL / FRAMES_PER_SEC_));
    }
    ::vTaskDelete(nullptr);
}

void TFTDisplay::clear() const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->fillScreen(TFTColors::BLACK);
    updated_ = true;
}

void TFTDisplay::clear(const uint8_t row) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->fillRect(0, HEIGHT_ * row / 10, WIDTH_, HEIGHT_ / 10, TFTColors::BLACK);
    updated_ = true;
}

void TFTDisplay::set_cursor_line(const uint8_t row, const uint8_t column) const {
    p_framebuffer_->setCursor(WIDTH_ * column / 28, HEIGHT_ * row / 10);
}

void TFTDisplay::set_cursor(const int16_t x, const int16_t y) const {
    p_framebuffer_->setCursor(x, y);
}

void TFTDisplay::set_text_size(const uint8_t size) const {
    p_framebuffer_->setTextSize(size);
}

void TFTDisplay::set_text_color(const uint16_t color) const {
    p_framebuffer_->setTextColor(color);
}

void TFTDisplay::set_text_wrap(bool wrap) const {
    p_framebuffer_->setTextWrap(wrap);
}

void TFTDisplay::set_backlight(const float brightness) const {
    if (brightness < 0.f || brightness > 100.f) {
        return;
    }
    if constexpr (CtBotConfig::MAINBOARD_REVISION >= 9'002) {
        p_backl_pwm_->off(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL));
        if constexpr (CtBotConfig::TFT_CONTROLLER_TYPE == 9341) {
            p_backl_pwm_->set_pwm(
                static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL), static_cast<int>(255.f * brightness / 100.f));
        } else {
            p_backl_pwm_->set_pwm(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL),
                static_cast<int>(255.f * (100.f - brightness) / 100.f));
        }
        p_backl_pwm_->on(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL));
    } else if constexpr (CtBotConfig::MAINBOARD_REVISION == 9'000) {
        arduino::analogWrite(CtBotConfig::TFT_BACKLIGHT_PIN, static_cast<int>(65535.f - (65535.f * brightness / 100.f)));
    }
}

uint8_t TFTDisplay::print(const char c, bool clear) const {
    if (clear) {
        int16_t x, y;
        uint16_t w {}, h {};
        char tmp[2] { c, '\0' };
        p_framebuffer_->getTextBounds(tmp, 0, 0, &x, &y, &w, &h);
        fill_rect(get_cursor_x(), get_cursor_y(), w, h, TFTColors::BLACK);
    }
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    const auto res { p_framebuffer_->write(static_cast<uint8_t>(c)) };
    updated_ = true;
    return res;
}

uint8_t TFTDisplay::print(const std::string& str, bool clear) const {
    if (clear) {
        int16_t x, y;
        uint16_t w {}, h {};
        p_framebuffer_->getTextBounds(str.c_str(), 0, 0, &x, &y, &w, &h);
        fill_rect(get_cursor_x(), get_cursor_y(), w, h, TFTColors::BLACK);
    }
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    const auto res { p_framebuffer_->Print::write(str.data(), str.length()) };
    updated_ = true;
    return res;
}

uint8_t TFTDisplay::print(const std::string* p_str, bool clear) const {
    const auto ret { print(*p_str, clear) };
    delete p_str;
    return ret;
}

int16_t TFTDisplay::get_cursor_x() const {
    return p_framebuffer_->getCursorX();
}

int16_t TFTDisplay::get_cursor_y() const {
    return p_framebuffer_->getCursorY();
}

void TFTDisplay::get_text_bounds(const std::string& str, const int16_t x, const int16_t y, int16_t* p_x, int16_t* p_y, uint16_t* p_w, uint16_t* p_h) const {
    p_framebuffer_->getTextBounds(str.c_str(), x, y, p_x, p_y, p_w, p_h);
}

void TFTDisplay::fill_screen(const uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->fillScreen(color);
    updated_ = true;
}

void TFTDisplay::fill_rect(const int16_t x, const int16_t y, const int16_t w, const int16_t h, const uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->fillRect(x, y, w, h, color);
    updated_ = true;
}

void TFTDisplay::draw_line(const int16_t x0, const int16_t y0, const int16_t x1, const int16_t y1, const uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->drawLine(x0, y0, x1, y1, color);
    updated_ = true;
}

void TFTDisplay::draw_rect(const int16_t x, const int16_t y, const int16_t w, const int16_t h, const uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->drawRect(x, y, w, h, color);
    updated_ = true;
}

void TFTDisplay::draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->drawTriangle(x0, y0, x1, y1, x2, y2, color);
    updated_ = true;
}

void TFTDisplay::fill_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->fillTriangle(x0, y0, x1, y1, x2, y2, color);
    updated_ = true;
}

void TFTDisplay::draw_circle(const int16_t x, const int16_t y, const int16_t r, const uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->drawCircle(x, y, r, color);
    updated_ = true;
}

void TFTDisplay::fill_circle(const int16_t x, const int16_t y, const int16_t r, const uint16_t color) const {
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    p_framebuffer_->fillCircle(x, y, r, color);
    updated_ = true;
}

void TFTDisplay::draw_button(Adafruit_GFX_Button* button, bool invert) const { // TODO: better solution
    std::lock_guard<std::mutex> lock { fb_mutex_ };
    button->drawButton(invert);
    updated_ = true;
}

bool TFTDisplay::get_touch_point(int16_t& x, int16_t& y, int16_t& z) const {
    if (!p_touch_point_) {
        return false;
    }

    std::lock_guard<std::mutex> lock { touch_mutex_ };
    auto point { *p_touch_point_ };
    x = point.x;
    y = point.y;
    z = point.z;
    return point.z;
}

} // namespace ctbot

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
 * @file    gfx_test.cpp
 * @brief   Test classes for display tests
 * @author  Timo Sandmann
 * @date    13.04.2019
 */

#include "tests.h"

#include "ctbot.h"
#include "scheduler.h"
#include "timer.h"

#include "driver/tft_display.h"

#include "Adafruit_GFX.h"
#include "arduino_freertos.h"
#include "pprintpp.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>


namespace freertos {
std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage();
}

namespace ctbot {
namespace tests {

void ButtonTest::initialize_buttons() {
    static_assert(CtBotConfig::TFT_AVAILABLE || (!CtBotConfig::BUTTON_TEST_AVAILABLE));

    auto p_tft { ctbot_.get_tft() };

    buttons_.emplace_back(new Adafruit_GFX_Button);
    buttons_[0]->initButtonUL(
        p_tft->get_context(), 0, 0 * BUTTON_H, BUTTON_W, BUTTON_H, TFTColors::BLACK, TFTColors::BLUE, TFTColors::WHITE, const_cast<char*>(PSTR("Lines")), 2);

    buttons_.emplace_back(new Adafruit_GFX_Button);
    buttons_[1]->initButtonUL(
        p_tft->get_context(), 0, 1 * BUTTON_H, BUTTON_W, BUTTON_H, TFTColors::BLACK, TFTColors::GREEN, TFTColors::WHITE, const_cast<char*>(PSTR("Rects")), 2);

    buttons_.emplace_back(new Adafruit_GFX_Button);
    buttons_[2]->initButtonUL(
        p_tft->get_context(), 0, 2 * BUTTON_H, BUTTON_W, BUTTON_H, TFTColors::BLACK, TFTColors::RED, TFTColors::WHITE, const_cast<char*>(PSTR("Circles")), 2);

    buttons_.emplace_back(new Adafruit_GFX_Button);
    buttons_[3]->initButtonUL(
        p_tft->get_context(), 0, 3 * BUTTON_H, BUTTON_W, BUTTON_H, TFTColors::BLACK, TFTColors::NAVY, TFTColors::WHITE, const_cast<char*>(PSTR("Tasks")), 2);
}

void ButtonTest::draw_buttons() {
    for (auto& b : buttons_) {
        ctbot_.get_tft()->draw_button(b);
    }
}

void ButtonTest::center_cursor_set(const std::string& str, const int16_t x, const int16_t y) {
    int16_t dummy_x, dummy_y;
    uint16_t w, h;
    ctbot_.get_tft()->get_text_bounds(str, 0, 0, &dummy_x, &dummy_y, &w, &h);
    ctbot_.get_tft()->set_cursor(x - w / 2, y - h / 2);
}

void ButtonTest::center_print(const std::string& str, bool clear) {
    center_cursor_set(str, DISPLAY_TEXTOFFSET + (ctbot_.get_tft()->get_width() - DISPLAY_TEXTOFFSET) / 2,
        DISPLAY_YOFFSET + (ctbot_.get_tft()->get_height() - DISPLAY_YOFFSET) / 2);
    ctbot_.get_tft()->print(str, clear);
}

ButtonTest::Buttons ButtonTest::button_release() {
    int16_t x { -1 };
    int16_t y { -1 };
    int16_t z { -1 };

    ctbot_.get_tft()->get_touch_point(x, y, z);

    // Scale from ~0->4000 to tft.width using the calibration #'s
    if (z != -1) {
        // ctbot_.get_comm()->debug_printf<true>(PP_ARGS("Touch raw: x={}\ty={}\tz={}\r\n", x, y, z));
        const int16_t px { x };
        const int16_t py { y };
        x = arduino::map<int16_t>(py, TS_MINY, TS_MAXY, 0, ctbot_.get_tft()->get_width());
        y = arduino::map<int16_t>(px, TS_MINX, TS_MAXX, 0, ctbot_.get_tft()->get_height());
        // ctbot_.get_comm()->debug_printf<true>(PP_ARGS("Touch: x={}\ty={}\tz={}\r\n", x, y, z));
    }

    // go through all the buttons, checking if they were pressed
    for (auto& b : buttons_) {
        if (b->contains(x, y)) {
            b->press(true); // tell the button it is pressed
        } else {
            b->press(false); // tell the button it is NOT pressed
        }
    }

    // now we can ask the buttons if their state has changed
    uint8_t btn {};
    for (uint8_t i {}; i < buttons_.size(); ++i) {
        if (buttons_[i]->justReleased()) {
            ctbot_.get_tft()->draw_button(buttons_[i]); // draw normal
            btn = i + 1;
        }

        if (buttons_[i]->justPressed()) {
            ctbot_.get_tft()->draw_button(buttons_[i], true); // draw invert
        }
    }

    return static_cast<Buttons>(btn);
}

void ButtonTest::run() {
    using namespace std::chrono_literals;

    if (!ctbot_.get_ready()) {
        return;
    }

    const auto btn { button_release() };
    switch (btn) {
        case Buttons::BTN_1: {
            ctbot_.get_tft()->set_text_color(TFTColors::BLUE);
            center_print(PSTR("Test Lines"), true);

            std::this_thread::sleep_for(750ms);

            for (uint8_t i { 0 }; i < 10; ++i) {
                test_lines(TFTColors::BLUE);
                std::this_thread::sleep_for(250ms);
            }

            break;
        }

        case Buttons::BTN_2: {
            ctbot_.get_tft()->set_text_color(TFTColors::GREEN);
            center_print(PSTR("Test Rects"), true);

            std::this_thread::sleep_for(750ms);

            for (uint8_t i { 0 }; i < 10; ++i) {
                test_rects(TFTColors::GREEN);
                std::this_thread::sleep_for(250ms);
            }

            break;
        }

        case Buttons::BTN_3: {
            ctbot_.get_tft()->set_text_color(TFTColors::RED);
            center_print(PSTR("Test Circles"), true);

            std::this_thread::sleep_for(750ms);

            for (uint8_t i { 0 }; i < 10; ++i) {
                test_circles(10, TFTColors::RED);
                std::this_thread::sleep_for(250ms);
            }

            break;
        }

        case Buttons::BTN_4: {
            display_tasks();
            break;

            // ctbot_.get_tft()->set_text_color(TFTColors::WHITE);
            // center_print("Test Triangles");

            // std::this_thread::sleep_for(750ms);

            // for (uint8_t i { 0 }; i < 10; ++i) {
            //     test_triangles();
            //     std::this_thread::sleep_for(250ms);
            // }

            // std::this_thread::sleep_for(500ms);

            // break;
        }

        default: return;
    }

    ctbot_.get_tft()->fill_screen(TFTColors::BLACK);
    draw_buttons();
}

void ButtonTest::test_lines(uint16_t color) {
    using namespace std::chrono_literals;

    ctbot_.get_tft()->clear();

    const auto w { ctbot_.get_tft()->get_width() };
    const auto h { ctbot_.get_tft()->get_height() };

    int x1 {};
    int y1 {};
    int y2 { h - 1 };
    int x2;

    for (x2 = 0; x2 < w; x2 += 6) {
        ctbot_.get_tft()->draw_line(x1, y1, x2, y2, color);
    }

    x2 = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) {
        ctbot_.get_tft()->draw_line(x1, y1, x2, y2, color);
    }
}

void ButtonTest::test_rects(uint16_t color) {
    ctbot_.get_tft()->clear();

    const int cx { ctbot_.get_tft()->get_width() / 2 };
    const int cy { ctbot_.get_tft()->get_height() / 2 };
    const auto n { std::min(ctbot_.get_tft()->get_width(), ctbot_.get_tft()->get_height()) };

    for (int i { 2 }; i < n; i += 6) {
        const auto i2 { i / 2 };
        ctbot_.get_tft()->draw_rect(cx - i2, cy - i2, i, i, color);
    }
}

void ButtonTest::test_circles(uint8_t radius, uint16_t color) {
    ctbot_.get_tft()->clear();

    const auto r2 { radius * 2 };
    const auto w { ctbot_.get_tft()->get_width() + radius };
    const auto h { ctbot_.get_tft()->get_height() + radius };

    for (int16_t x { 0 }; x < w; x += r2) {
        for (int16_t y { 0 }; y < h; y += r2) {
            ctbot_.get_tft()->draw_circle(x, y, radius, color);
        }
    }
}

void ButtonTest::test_triangles() {
    ctbot_.get_tft()->clear();

    const auto cx { ctbot_.get_tft()->get_width() / 2 - 1 };
    const auto cy { ctbot_.get_tft()->get_height() / 2 - 1 };
    const auto n { std::min(cx, cy) };

    for (int i { 0 }; i < n; i += 5) {
        ctbot_.get_tft()->draw_triangle(cx, cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            TFTColors::color565(200, 200, i + 50));
    }
}

ButtonTest::ButtonTest(CtBot& ctbot) : ctbot_(ctbot) {
    initialize_buttons();

    ctbot_.get_scheduler()->task_add(PSTR("TFT-Test"), TASK_PERIOD_MS, 2, 8192, [this]() {
        ctbot_.get_tft()->fill_screen(TFTColors::BLACK);
        draw_buttons();

        run();
    });
}

ButtonTest::~ButtonTest() {
    for (auto& b : buttons_) {
        delete b;
    }
}

void ButtonTest::display_tasks() {
    using namespace std::chrono_literals;

    ctbot_.get_tft()->clear();

    uint32_t last_touch_cnt { ctbot_.get_tft()->get_touch_counter() };
    do { // FIXME: don't loop here in method
        auto p_runtime_stats { ctbot_.get_scheduler()->get_runtime_stats() };

        ctbot_.get_tft()->set_text_size(2);
        ctbot_.get_tft()->set_text_color(TFTColors::RED);

        const int16_t tft_w { static_cast<int16_t>(ctbot_.get_tft()->get_width() - 20) };
        int16_t y { 5 };
        for (auto& e : *p_runtime_stats) {
            if (e.second < 0.1f) {
                ctbot_.get_tft()->fill_rect(0, y, tft_w, ctbot_.get_tft()->get_height() - (y + 50), TFTColors::BLACK);
                break;
            }

            const int16_t w { static_cast<int16_t>(e.second * tft_w / 100.f) };
            ctbot_.get_tft()->fill_rect(10, y, w, 20, TFTColors::GREEN);
            ctbot_.get_tft()->fill_rect(w, y, tft_w, 20, TFTColors::BLACK);
            ctbot_.get_tft()->set_cursor(20, y + 2);
            ctbot_.get_tft()->printf(PP_ARGS("{s}: {.2} %%", ::pcTaskGetName(e.first), e.second));
            y += 25;
            if (y > 175) {
                break;
            }
        }

        static uint32_t last_ms {};
        const auto now { Timer::get_ms() };
        if (now - last_ms > 2'000) {
            last_ms = now;

            const int16_t y { static_cast<int16_t>(ctbot_.get_tft()->get_height() - 50) };
            ctbot_.get_tft()->set_text_size(2);
            ctbot_.get_tft()->set_text_color(TFTColors::PINK);

            const auto info { freertos::ram1_usage() };
            const float ram_size_f { static_cast<float>(std::get<3>(info)) };
            const size_t ram_size_kb { std::get<3>(info) / 1024UL };
            const size_t ram_used { std::get<3>(info) - std::get<0>(info) };
            {
                const int16_t w { static_cast<int16_t>(static_cast<float>(ram_used) / ram_size_f * tft_w) };
                ctbot_.get_tft()->fill_rect(10, y, w, 20, TFTColors::BLUE);
                ctbot_.get_tft()->fill_rect(w, y, tft_w, 20, TFTColors::BLACK);
                ctbot_.get_tft()->set_cursor_line(8, 2);
                ctbot_.get_tft()->printf(PP_ARGS("used RAM: {} KB / {} KB", ram_used / 1024UL, ram_size_kb));
            }
            {
                const int16_t w { static_cast<int16_t>(static_cast<float>(std::get<1>(info)) / ram_size_f * tft_w) };
                ctbot_.get_tft()->fill_rect(10, y + 25, w, 20, TFTColors::BLUE);
                ctbot_.get_tft()->fill_rect(w, y + 25, tft_w, 20, TFTColors::BLACK);
                ctbot_.get_tft()->set_cursor_line(9, 2);
                ctbot_.get_tft()->printf(PP_ARGS("used heap: {} KB", std::get<1>(info) / 1024UL));
            }
        }

        // for (size_t i { 0 }; i < 20; ++i) {
        //     std::this_thread::sleep_for(50ms);
        //     if (ctbot_.get_tft()->touched()) {
        //         return;
        //     }
        // }
        std::this_thread::sleep_for(200ms);
    } while (last_touch_cnt == ctbot_.get_tft()->get_touch_counter());
}

} // namespace tests
} // namespace ctbot

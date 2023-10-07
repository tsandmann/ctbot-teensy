/*
 * This file is part of the ct-Bot teensy framework.
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
 * @file    tests.cpp
 * @brief   Test classes implementing tasks to test functionality
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "tests.h"

#include "ctbot.h"
#include "scheduler.h"
#include "sensors.h"

#include "driver/ena_i2c.h"
#include "driver/lc_display.h"
#include "driver/leds_i2c.h"
#include "driver/tft_display.h"
#include "driver/spi_t4.h"

#include "XPT2046_Touchscreen.h"

#include "arduino_freertos.h"
#include "pprintpp.hpp"
#include "thread_gthread.h"

#include <chrono>
#include <cstring>


namespace ctbot {
namespace tests {

BlinkTest::BlinkTest(CtBot& ctbot) : ctbot_ { ctbot }, state_ {} {
    arduino::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    ctbot_.get_scheduler()->task_add(PSTR("blinktest"), TASK_PERIOD_MS, 512UL, [this]() { return run(); });
}

BlinkTest::~BlinkTest() = default;

void BlinkTest::run() {
    arduino::digitalWriteFast(arduino::LED_BUILTIN, state_);
    state_ = !state_;
}


LedTest::LedTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot_.get_scheduler()->task_add(PSTR("ledtest"), TASK_PERIOD_MS, 512UL, [this]() { return run(); });
}

LedTest::~LedTest() = default;

void LedTest::run() {
    static uint8_t led_idx { 7 };

    LedTypes led(static_cast<LedTypes>(1 << led_idx));
    ctbot_.get_leds()->off(led);

    if (++led_idx > 7) {
        led_idx = 0;
    }

    led = static_cast<LedTypes>(1 << led_idx);
    ctbot_.get_leds()->on(led);
}


LcdTest::LcdTest(CtBot& ctbot) : ctbot_(ctbot), x_ {} {
    ctbot_.get_scheduler()->task_add(PSTR("lcdtest"), TASK_PERIOD_MS, [this]() { return run(); });
}

LcdTest::~LcdTest() = default;

void LcdTest::run() {
    ctbot_.get_lcd()->set_cursor((x_ % 80) / 20 + 1, x_ % 20 + 1);
    ctbot_.get_lcd()->print(' ');
    ++x_;
    ctbot_.get_lcd()->set_cursor((x_ % 80) / 20 + 1, x_ % 20 + 1);
    ctbot_.get_lcd()->print('*');
    ctbot_.get_lcd()->set_cursor(((x_ + 20) % 80) / 20 + 1, 1);
    ctbot_.get_lcd()->print(PSTR("Hello World :-)"));
    ctbot_.get_lcd()->set_cursor(((x_ + 40) % 80) / 20 + 1, 1);
    ctbot_.get_lcd()->printf("%5u", static_cast<uint16_t>(x_));
}


EnaTest::EnaTest(CtBot& ctbot) : ctbot_ { ctbot }, ena_idx_ {} {
    ctbot_.get_scheduler()->task_add(PSTR("enatest"), TASK_PERIOD_MS, [this]() { return run(); });
}

EnaTest::~EnaTest() = default;

void EnaTest::run() {
    ctbot_.get_ena()->set(static_cast<EnaI2cTypes>(~(1 << ena_idx_++)));
    if (ena_idx_ > 7) {
        ena_idx_ = 0;
    }
}


SensorLcdTest::SensorLcdTest(CtBot& ctbot) : ctbot_(ctbot), running_ { true } {
    ctbot.get_scheduler()->task_add(PSTR("senstest"), TASK_PERIOD_MS, 1'024UL, [this]() { return run(); });
}

SensorLcdTest::~SensorLcdTest() {
    running_ = false;
}

void SensorLcdTest::run() {
    if (!running_ || !ctbot_.get_ready()) {
        return;
    }

    auto const p_sens { ctbot_.get_sensors() };

    ctbot_.get_lcd()->set_cursor(1, 1);
    ctbot_.get_lcd()->printf(PSTR("D=%4d %4d"), p_sens->get_distance_l(), p_sens->get_distance_r());

    ctbot_.get_lcd()->set_cursor(2, 1);
    ctbot_.get_lcd()->printf(PSTR("B=%03X %03X L=%03X %03X "), p_sens->get_border_l(), p_sens->get_border_r(), p_sens->get_line_l(), p_sens->get_line_r());

    ctbot_.get_lcd()->set_cursor(3, 1);
    ctbot_.get_lcd()->printf(
        PSTR("R=%2d %2d T=%d "), std::labs(p_sens->get_enc_l().get()) % 100, std::labs(p_sens->get_enc_r().get()) % 100, p_sens->get_transport());

    ctbot_.get_lcd()->set_cursor(4, 1);
    ctbot_.get_lcd()->printf(PSTR("S=%4d %4d  "), static_cast<int16_t>(p_sens->get_enc_l().get_speed()), static_cast<int16_t>(p_sens->get_enc_r().get_speed()));
    ctbot_.get_lcd()->set_cursor(4, 14);
    ctbot_.get_lcd()->printf(PSTR("RC=0x%02x"), static_cast<int16_t>(p_sens->get_rc5().get_cmd()));
}


TaskWaitTest::TaskWaitTest(CtBot& ctbot) : ctbot_(ctbot), running_ { true } {
    using namespace std::chrono;
    using namespace std::chrono_literals;

    const auto last_stack_size { free_rtos_std::gthr_freertos::set_next_stacksize(1024) };
    p_thr1_ = new std::thread([this]() {
        while (running_) {
            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                const auto free_stack { ctbot_.get_scheduler()->get_free_stack() };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 1: got condition 1 at {} ms. stack free: {} byte.\r\n", now_ms, free_stack));
                ctbot_.get_comm()->flush();
            }

            std::this_thread::sleep_for(10s);

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 1: notify condition 2 at {} ms...\r\n", now_ms));
                ctbot_.get_comm()->flush();
            }

            cv2_.notify_all();

            ctbot_.get_comm()->debug_print(PSTR("TaskWaitTest Thr 1: waiting for condition 1...\r\n"), true);
            ctbot_.get_comm()->flush();
            std::unique_lock<std::mutex> lk(m1_);
            cv1_.wait(lk);
        }
    });
    free_rtos_std::gthr_freertos::set_name(p_thr1_, PSTR("wait_1"));

    free_rtos_std::gthr_freertos::set_next_stacksize(768);
    p_thr2_ = new std::thread([this]() {
        while (running_) {
            ctbot_.get_comm()->debug_print(PSTR("TaskWaitTest Thr 2: waiting for condition 2...\r\n"), true);
            ctbot_.get_comm()->flush();
            {
                std::unique_lock<std::mutex> lk(m2_);
                cv2_.wait(lk);
            }

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                const auto free_stack { ctbot_.get_scheduler()->get_free_stack() };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 2: got condition 2 at {} ms. stack free: {} byte.\r\n", now_ms, free_stack));
                ctbot_.get_comm()->flush();
            }

            std::this_thread::sleep_for(5s);

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 2: notify condition 1 at {} ms...\r\n", now_ms));
                ctbot_.get_comm()->flush();
            }

            cv1_.notify_all();
        }
    });
    free_rtos_std::gthr_freertos::set_name(p_thr2_, "wait_2");

    p_thr3_ = new std::thread([this]() {
        while (running_) {
            ctbot_.get_comm()->debug_print(PSTR("TaskWaitTest Thr 3: waiting for condition 2...\r\n"), true);
            ctbot_.get_comm()->flush();
            {
                std::unique_lock<std::mutex> lk(m2_);
                cv2_.wait(lk);
            }

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                const auto free_stack { ctbot_.get_scheduler()->get_free_stack() };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 3: got condition 2 at {} ms. stack free: {} byte.\r\n", now_ms, free_stack));
                ctbot_.get_comm()->flush();
            }
        }
    });
    free_rtos_std::gthr_freertos::set_name(p_thr3_, PSTR("wait_3"));

    free_rtos_std::gthr_freertos::set_next_stacksize(last_stack_size);

    ctbot_.get_scheduler()->task_register(PSTR("wait_1"));
    ctbot_.get_scheduler()->task_register(PSTR("wait_2"));
}

TaskWaitTest::~TaskWaitTest() {
    ctbot_.get_comm()->debug_print(PSTR("TaskWaitTest::~TaskWaitTest(): exiting...\r\n"), true);
    ctbot_.get_comm()->flush();

    running_ = false;

    if (p_thr1_ && p_thr1_->joinable()) {
        p_thr1_->join();
        delete p_thr1_;
    }
    if (p_thr2_ && p_thr2_->joinable()) {
        p_thr2_->join();
        delete p_thr2_;
    }
    if (p_thr3_ && p_thr3_->joinable()) {
        p_thr3_->join();
        delete p_thr3_;
    }
}


TftTest::TftTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot_.get_tft()->fill_screen(TFTColors::BLACK);
    ctbot_.get_scheduler()->task_add(PSTR("TFT-Test"), TASK_PERIOD_MS, 2, 2'048, [this]() { return run(); });
}

void TftTest::run() {
    using namespace std::chrono_literals;
    // auto& comm { *ctbot_.get_comm() };

    // comm.debug_print(PSTR("\r\nBenchmark                Time\r\n"), true);

    // {
    // const auto start { arduino::micros() };
    // ctbot_.get_tft()->fill_screen(TFTColors::BLACK);
    // const auto t { arduino::micros() - start };
    // comm.debug_printf<true>(PP_ARGS("Screen fill              {.2} ms\r\n", t / 1'000.f));
    // }
    // std::this_thread::sleep_for(250ms);

    // comm.debug_printf<true>(PP_ARGS("Text                     {.2} ms\r\n", testText() / 1'000.f));
    for (int16_t i {}; i < 320; i += 20) {
        testText(i, 0);
        std::this_thread::sleep_for(40ms);
    }

    // comm.debug_printf<true>(PP_ARGS("Lines                    {.2} ms\r\n", testLines(TFTColors::CYAN) / 1'000.f));
    testLines(TFTColors::CYAN);
    std::this_thread::sleep_for(500ms);

    // comm.debug_printf<true>(PP_ARGS("Rectangles (outline)     {.2} ms\r\n", testRects(TFTColors::GREEN) / 1'000.f));
    testRects(TFTColors::GREEN);
    std::this_thread::sleep_for(500ms);

    ctbot_.get_tft()->fill_screen(TFTColors::BLACK);
    // comm.debug_printf<true>(PP_ARGS("Circles (outline)        {.2} ms\r\n", testCircles(10, TFTColors::RED) / 1'000.f));
    testCircles(10, TFTColors::RED);
    std::this_thread::sleep_for(500ms);

    testFilledCircles(10, TFTColors::MAGENTA);
    std::this_thread::sleep_for(500ms);

    // comm.debug_printf<true>(PP_ARGS("Triangles (outline)      {.2} ms\r\n", testTriangles() / 1'000.f));
    testTriangles();
    std::this_thread::sleep_for(500ms);

    // comm.debug_printf<true>(PP_ARGS("Triangles (filled)       {.2} ms\r\n", testFilledTriangles() / 1'000.f));
    testFilledTriangles();
    std::this_thread::sleep_for(500ms);
}

unsigned long TftTest::testText(int16_t col, int16_t row) {
    auto p_tft { ctbot_.get_tft() };

    const unsigned long start { arduino::micros() };
    p_tft->set_text_wrap(false);
    p_tft->fill_screen(TFTColors::BLACK);
    p_tft->set_cursor(col, row);
    p_tft->set_text_color(TFTColors::WHITE);
    p_tft->set_text_size(1);
    p_tft->print(PSTR("Hello World!\r\n"));
    p_tft->set_text_color(TFTColors::YELLOW);
    p_tft->set_text_size(2);
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->printf(PSTR("%f\r\n"), 1234.56f);
    p_tft->set_text_color(TFTColors::RED);
    p_tft->set_text_size(3);
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->printf(PSTR("0x%x\r\n"), 0xDEADBEEF);
    p_tft->set_text_color(TFTColors::GREEN);
    p_tft->set_text_size(5);
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("Groop\r\n"));
    p_tft->set_text_size(2);
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("I implore thee,\r\n"));
    p_tft->set_text_size(1);
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("my foonting turlingdromes.\r\n"));
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("And hooptiously drangle me\r\n"));
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("with crinkly bindlewurdles,\r\n"));
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("Or I will rend thee\r\n"));
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("in the gobberwarts\r\n"));
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("with my blurglecruncheon,\r\n"));
    p_tft->set_cursor(col, p_tft->get_cursor_y());
    p_tft->print(PSTR("see if I don't!\r\n"));

    // p_tft->set_text_color(TFTColors::WHITE);
    // p_tft->set_cursor(col, p_tft->get_cursor_y());
    // p_tft->set_text_wrap(true);
    // p_tft->print(
    //     PSTR("Alice was beginning to get very tired of sitting by her sister on the bank, and of having nothing to do: once or twice she had peeped into "
    //          "the book her sister was reading, but it had no pictures or conversations in it, 'and what is the use of a book,' thought Alice 'without "
    //          "pictures or conversations?'\r\n"));

    return arduino::micros() - start;
}

unsigned long TftTest::testLines(uint16_t color) {
    auto p_tft { ctbot_.get_tft() };

    unsigned long t;
    int x1, y1, x2, y2, w = p_tft->get_width(), h = p_tft->get_height();

    p_tft->fill_screen(TFTColors::BLACK);

    x1 = y1 = 0;
    y2 = h - 1;
    const unsigned long start { arduino::micros() };
    for (x2 = 0; x2 < w; x2 += 6) {
        p_tft->draw_line(x1, y1, x2, y2, color);
    }
    x2 = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) {
        p_tft->draw_line(x1, y1, x2, y2, color);
    }

    t = arduino::micros() - start; // fill_screen doesn't count against timing

    return t;
}

unsigned long TftTest::testFastLines(uint16_t color1, uint16_t color2) {
    auto p_tft { ctbot_.get_tft() };

    int x, y, w = p_tft->get_width(), h = p_tft->get_height();

    p_tft->fill_screen(TFTColors::BLACK);
    const unsigned long start { arduino::micros() };
    for (y = 0; y < h; y += 5) {
        p_tft->draw_line(0, y, 0, w, color1);
    }
    for (x = 0; x < w; x += 5) {
        p_tft->draw_line(x, 0, x, h, color2);
    }

    return arduino::micros() - start;
}

unsigned long TftTest::testRects(uint16_t color) {
    auto p_tft { ctbot_.get_tft() };

    int n, i, i2, cx = p_tft->get_width() / 2, cy = p_tft->get_height() / 2;

    p_tft->fill_screen(TFTColors::BLACK);
    n = std::min(p_tft->get_width(), p_tft->get_height());
    const unsigned long start { arduino::micros() };
    for (i = 2; i < n; i += 6) {
        i2 = i / 2;
        p_tft->draw_rect(cx - i2, cy - i2, i, i, color);
    }

    return arduino::micros() - start;
}

unsigned long TftTest::testFilledRects(uint16_t color1, uint16_t color2) {
    auto p_tft { ctbot_.get_tft() };

    unsigned long t = 0;
    int n, i, i2, cx = p_tft->get_width() / 2 - 1, cy = p_tft->get_height() / 2 - 1;

    p_tft->fill_screen(TFTColors::BLACK);
    n = std::min(p_tft->get_width(), p_tft->get_height());
    for (i = n; i > 0; i -= 6) {
        i2 = i / 2;
        const unsigned long start { arduino::micros() };
        p_tft->fill_rect(cx - i2, cy - i2, i, i, color1);
        // Outlines are not included in timing results
        t += arduino::micros() - start;
        p_tft->draw_rect(cx - i2, cy - i2, i, i, color2);
    }

    return t;
}

unsigned long TftTest::testFilledCircles(uint8_t radius, uint16_t color) {
    auto p_tft { ctbot_.get_tft() };

    int x, y, w = p_tft->get_width(), h = p_tft->get_height(), r2 = radius * 2;

    p_tft->fill_screen(TFTColors::BLACK);
    const unsigned long start { arduino::micros() };
    for (x = radius; x < w; x += r2) {
        for (y = radius; y < h; y += r2) {
            p_tft->fill_circle(x, y, radius, color);
        }
    }

    return arduino::micros() - start;
}

unsigned long TftTest::testCircles(uint8_t radius, uint16_t color) {
    auto p_tft { ctbot_.get_tft() };

    int x, y, r2 = radius * 2, w = p_tft->get_width() + radius, h = p_tft->get_height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    const unsigned long start { arduino::micros() };
    for (x = 0; x < w; x += r2) {
        for (y = 0; y < h; y += r2) {
            p_tft->draw_circle(x, y, radius, color);
        }
    }

    return arduino::micros() - start;
}

unsigned long TftTest::testTriangles() {
    auto p_tft { ctbot_.get_tft() };

    int n, i, cx = p_tft->get_width() / 2 - 1, cy = p_tft->get_height() / 2 - 1;

    p_tft->fill_screen(TFTColors::BLACK);
    n = std::min(cx, cy);
    const unsigned long start { arduino::micros() };
    for (i = 0; i < n; i += 5) {
        p_tft->draw_triangle(cx, cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            TFTColorHelper::color565(200, 20, i));
    }

    return arduino::micros() - start;
}

unsigned long TftTest::testFilledTriangles() {
    auto p_tft { ctbot_.get_tft() };

    unsigned long t {};
    int i, cx = p_tft->get_width() / 2 - 1, cy = p_tft->get_height() / 2 - 1;

    p_tft->fill_screen(TFTColors::BLACK);
    for (i = std::min(cx, cy); i > 10; i -= 5) {
        const unsigned long start { arduino::micros() };
        p_tft->fill_triangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, TFTColorHelper::color565(0, i, i));
        t += arduino::micros() - start;
        p_tft->draw_triangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, TFTColorHelper::color565(i, i, 0));
    }

    return t;
}

TftTest::~TftTest() {}


TouchTest::TouchTest(CtBot& ctbot) : ctbot_(ctbot), p_touch_ {} {
    Scheduler::enter_critical_section();
    p_touch_ = new XPT2046_Touchscreen { CtBotConfig::TFT_TOUCH_CS_PIN, freertos::get_spi<CtBotConfig::TFT_SPI - 1>() };
    if (!p_touch_) {
        Scheduler::exit_critical_section();
        return;
    }

    const auto touch_res { p_touch_->begin() };
    Scheduler::exit_critical_section();
    p_touch_->setRotation(CtBotConfig::TFT_TOUCH_ROTATION);

    auto& comm { *ctbot_.get_comm() };
    if (touch_res) {
        // comm.debug_printf<true>(PP_ARGS("Touch controller {#x} detected.\r\n", p_touch_->getVersion()));
        comm.debug_print(PSTR("Touch controller detected.\r\n"), true);
    } else {
        comm.debug_print(PSTR("Touch controller init failed.\r\n"), true);
    }

    ctbot_.get_scheduler()->task_add(PSTR("touchtest"), TASK_PERIOD_MS, 1, 1'024, [this]() { return run(); });
}

void TouchTest::run() {
    auto& comm { *ctbot_.get_comm() };

    const auto start { arduino::micros() };
    if (p_touch_->touched()) {
        uint16_t x, y;
        uint8_t z;
        p_touch_->readData(&x, &y, &z);
        const auto dt { arduino::micros() - start };

        comm.debug_printf<true>(PP_ARGS(
            "Touch event: x={}\ty={}, took {} us\r\n", arduino::map<uint16_t>(x, 0U, 4'095U, 0U, 240), arduino::map<uint16_t>(y, 0U, 4'095U, 0U, 320), dt));
    }
}

TouchTest::~TouchTest() {
    delete p_touch_;
}

} // namespace tests
} // namespace ctbot

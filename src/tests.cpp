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
 * @file    tests.cpp
 * @brief   Test classes implementing tasks to test functionality
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "tests.h"
#include "ctbot.h"
#include "scheduler.h"
#include "leds.h"
#include "ena.h"
#include "sensors.h"
#include "lc_display.h"

#include "arduino_fixed.h"
#include "pprintpp.hpp"
#include "thread_gthread.h"

#include <chrono>
#include <cstring>


namespace ctbot {
namespace tests {

BlinkTest::BlinkTest(CtBot& ctbot) : ctbot_ { ctbot }, state_ { false } {
    arduino::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    ctbot_.get_scheduler()->task_add("blinktest", TASK_PERIOD_MS, 192UL, [this]() { return run(); });
}

void BlinkTest::run() {
    arduino::digitalWriteFast(arduino::LED_BUILTIN, state_);
    state_ = !state_;
}


LedTest::LedTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot_.get_scheduler()->task_add("ledtest", TASK_PERIOD_MS, 512UL, [this]() { return run(); });
}

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


LcdTest::LcdTest(CtBot& ctbot) : ctbot_(ctbot), x_ { 0U } {
    ctbot_.get_scheduler()->task_add("lcdtest", TASK_PERIOD_MS, [this]() { return run(); });
}

void LcdTest::run() {
    ctbot_.get_lcd()->set_cursor((x_ % 80) / 20 + 1, x_ % 20 + 1);
    ctbot_.get_lcd()->print(' ');
    ++x_;
    ctbot_.get_lcd()->set_cursor((x_ % 80) / 20 + 1, x_ % 20 + 1);
    ctbot_.get_lcd()->print('*');
    ctbot_.get_lcd()->set_cursor(((x_ + 20) % 80) / 20 + 1, 1);
    ctbot_.get_lcd()->print("Hello World :-)");
    ctbot_.get_lcd()->set_cursor(((x_ + 40) % 80) / 20 + 1, 1);
    ctbot_.get_lcd()->printf("%5u", static_cast<uint16_t>(x_));
}


EnaTest::EnaTest(CtBot& ctbot) : ctbot_ { ctbot }, p_ena_ { new Ena }, ena_idx_ { 0 } {
    ctbot_.get_scheduler()->task_add("enatest", TASK_PERIOD_MS, [this]() { return run(); });
}

void EnaTest::run() {
    p_ena_->set(static_cast<EnaTypes>(~(1 << ena_idx_++)));
    if (ena_idx_ > 7) {
        ena_idx_ = 0;
    }
}


SensorLcdTest::SensorLcdTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot.get_scheduler()->task_add("senstest", TASK_PERIOD_MS, 1024UL, [this]() { return run(); });
}

void SensorLcdTest::run() {
    auto const p_sens(ctbot_.get_sensors());

    ctbot_.get_lcd()->set_cursor(1, 1);
    ctbot_.get_lcd()->printf("P%03X %03X D=%4d %4d", p_sens->get_ldr_l(), p_sens->get_ldr_r(), p_sens->get_distance_l(), p_sens->get_distance_r());

    ctbot_.get_lcd()->set_cursor(2, 1);
    ctbot_.get_lcd()->printf("B=%03X %03X L=%03X %03X ", p_sens->get_border_l(), p_sens->get_border_r(), p_sens->get_line_l(), p_sens->get_line_r());

    ctbot_.get_lcd()->set_cursor(3, 1);
    ctbot_.get_lcd()->printf("R=%2d %2d K=%d T=%d ", std::labs(p_sens->get_enc_l().get()) % 100, std::labs(p_sens->get_enc_r().get()) % 100,
        p_sens->get_shutter(), p_sens->get_transport());

    ctbot_.get_lcd()->set_cursor(4, 1);
    ctbot_.get_lcd()->printf("S=%4d %4d  ", static_cast<int16_t>(p_sens->get_enc_l().get_speed()), static_cast<int16_t>(p_sens->get_enc_r().get_speed()));
    ctbot_.get_lcd()->set_cursor(4, 14);
    ctbot_.get_lcd()->printf("RC=0x%02x", static_cast<int16_t>(p_sens->get_rc5().get_cmd()));
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
            }

            std::this_thread::sleep_for(1s);

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 1: notify condition 2 at {} ms...\r\n", now_ms));
            }

            cv2_.notify_all();

            ctbot_.get_comm()->debug_print("TaskWaitTest Thr 1: waiting for condition 1...\r\n", true);
            std::unique_lock<std::mutex> lk(m1_);
            cv1_.wait(lk);
        }
    });
    { free_rtos_std::gthr_freertos::set_name(p_thr1_, "wait_1"); }

    free_rtos_std::gthr_freertos::set_next_stacksize(768);
    p_thr2_ = new std::thread([this]() {
        while (running_) {
            ctbot_.get_comm()->debug_print("TaskWaitTest Thr 2: waiting for condition 2...\r\n", true);
            {
                std::unique_lock<std::mutex> lk(m2_);
                cv2_.wait(lk);
            }

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                const auto free_stack { ctbot_.get_scheduler()->get_free_stack() };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 2: got condition 2 at {} ms. stack free: {} byte.\r\n", now_ms, free_stack));
            }

            std::this_thread::sleep_for(5s);

            {
                const auto now_ms { static_cast<uint32_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) };
                ctbot_.get_comm()->debug_printf<true>(PP_ARGS("TaskWaitTest Thr 2: notify condition 1 at {} ms...\r\n", now_ms));
            }

            cv1_.notify_all();
        }
    });
    { free_rtos_std::gthr_freertos::set_name(p_thr2_, "wait_2"); }

    free_rtos_std::gthr_freertos::set_next_stacksize(last_stack_size);

    ctbot_.get_scheduler()->task_register("wait_1");
    ctbot_.get_scheduler()->task_register("wait_2");
}

TaskWaitTest::~TaskWaitTest() {
    ctbot_.get_comm()->debug_print("TaskWaitTest::~TaskWaitTest(): exiting...\r\n", true);

    running_ = false;

    if (p_thr1_ && p_thr1_->joinable()) {
        p_thr1_->join();
        delete p_thr1_;
    }
    if (p_thr2_ && p_thr2_->joinable()) {
        p_thr2_->join();
        delete p_thr2_;
    }
}

} // namespace tests
} // namespace ctbot

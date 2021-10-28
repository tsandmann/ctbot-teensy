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
 * @file    ctbot_main.cpp
 * @brief   Entry point of ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "ctbot.h"
#include "ctbot_config.h"
#include "timer.h"
#include "scheduler.h"
#include "serial_io.h"
#include "tests.h"

#include "arduino_freertos.h"

#include <thread>
#include <memory>
#include <cstring>


static constexpr bool DEBUG { false };

namespace ctbot {
tests::BlinkTest* g_blink_test {};
tests::LedTest* g_led_test {};
tests::LcdTest* g_lcd_test {};
tests::EnaTest* g_ena_test {};
tests::SensorLcdTest* g_sensor_lcd_test {};
tests::TftTest* g_tft_test {};
tests::TouchTest* g_touch_test {};
tests::ButtonTest* g_button_test {};
tests::TaskWaitTest* g_wait_test {};
} // namespace ctbot

#ifndef EXC_PRINTF
#define EXC_PRINTF(...) ::serialport_puts(__VA_ARGS__)
#endif

/**
 * @brief Task to initialize everything
 * @note This task is suspended forever after initialization is done.
 */
static FLASHMEM void init_task(void*) {
    using namespace ctbot;

    /* wait for USB device enumeration, terminal program connection, etc. */
    ::vTaskDelay(pdMS_TO_TICKS(CtBotConfig::BOOT_DELAY_MS));

    EXC_PRINTF(PSTR("\r\nRunning FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ".\r\n"));

    if (DEBUG) {
        EXC_PRINTF(PSTR("init_task():\r\n"));
        freertos::print_ram_usage();
    }

    /* create CtBot singleton instance... */
    if (DEBUG) {
        EXC_PRINTF(PSTR("creating CtBot instance...\r\n"));
    }
    CtBot& ctbot { CtBot::get_instance() };

    /* initialize it... */
    if (DEBUG) {
        EXC_PRINTF(PSTR("calling ctbot.setup()...\r\n"));
    }
    ctbot.setup(true);
    if (DEBUG) {
        EXC_PRINTF(PSTR("ctbot.setup() done.\r\n"));
    }

    /* create test tasks if configured... */
    if (CtBotConfig::BLINK_TEST_AVAILABLE) {
        g_blink_test = new tests::BlinkTest { ctbot };
        std::atexit([]() { delete g_blink_test; });
    }

    if (CtBotConfig::LED_TEST_AVAILABLE) {
        g_led_test = new tests::LedTest { ctbot };
        std::atexit([]() { delete g_led_test; });
    }

    if (CtBotConfig::LCD_TEST_AVAILABLE) {
        g_lcd_test = new tests::LcdTest { ctbot };
        std::atexit([]() { delete g_lcd_test; });
    }

    if (CtBotConfig::ENA_TEST_AVAILABLE) {
        g_ena_test = new tests::EnaTest { ctbot };
        std::atexit([]() { delete g_ena_test; });
    }

    if (CtBotConfig::SENS_LCD_TEST_AVAILABLE) {
        g_sensor_lcd_test = new tests::SensorLcdTest { ctbot };
        std::atexit([]() { delete g_sensor_lcd_test; });
    }

    if (CtBotConfig::TFT_TEST_AVAILABLE) {
        g_tft_test = new tests::TftTest { ctbot };
        std::atexit([]() { delete g_tft_test; });
    }

    if (CtBotConfig::TOUCH_TEST_AVAILABLE) {
        g_touch_test = new tests::TouchTest { ctbot };
        std::atexit([]() { delete g_touch_test; });
    }

    if (CtBotConfig::BUTTON_TEST_AVAILABLE) {
        g_button_test = new tests::ButtonTest { ctbot };
        std::atexit([]() {
            EXC_PRINTF(PSTR("deleting g_button_test...\r\n"));
            delete g_button_test;
            EXC_PRINTF(PSTR("done.\r\n"));
        });
    }

    if (CtBotConfig::TASKWAIT_TEST_AVAILABLE) {
        g_wait_test = new tests::TaskWaitTest { ctbot };
        std::atexit([]() { delete g_wait_test; });
    }

    if (DEBUG) {
        ::vTaskDelay(pdMS_TO_TICKS(200));
        freertos::print_ram_usage();
    }

    arduino::digitalWriteFast(CtBotConfig::DEBUG_LED_PIN, false); // turn debug LED off

    if (DEBUG) {
        EXC_PRINTF(PSTR("deleting init task...\r\n"));
    }
    ::vTaskDelete(nullptr);
}

extern "C" {
/**
 * @brief Entry point for ct-Bot initialization
 */
FLASHMEM __attribute__((noinline)) void setup() {
    using namespace ctbot;
    arduino::pinMode(CtBotConfig::DEBUG_LED_PIN, arduino::OUTPUT);
    arduino::digitalWriteFast(CtBotConfig::DEBUG_LED_PIN, true); // turn debug LED on

    if (DEBUG) {
        EXC_PRINTF(PSTR("setup(): setting up init task...\r\n"));
    }
    ::xTaskCreate(init_task, PSTR("INIT"), 4096 / sizeof(StackType_t), nullptr, configMAX_PRIORITIES - 1, nullptr);

    if (DEBUG) {
        EXC_PRINTF(PSTR("setup(): calling ::vTaskStartScheduler()...\r\n"));
    }
    ::vTaskStartScheduler();

    freertos::error_blink(10);
    // we never ever get here
}

/**
 * @brief Arduino loop function, not used because of scheduler (@see Scheduler)
 */
FLASHMEM void loop() {}

/**
 * @brief libc "syscall" implementation for writing characters to a filestream
 * @param[in] ptr: Pointer to data to write
 * @param[in] len: Number of bytes to write
 * @return Number of bytes written, -1 in case of an error
 * @note File handle parameter is ignored, any data is written to Serial
 */
FLASHMEM int _write(int, char* ptr, int len) {
    using namespace ctbot;

    CtBot& ctbot { CtBot::get_instance() };
    arduino::SerialIO* p_serial { ctbot.get_serial_usb() };

    if (p_serial) {
        const auto n { p_serial->write(ptr, len) };
        p_serial->flush();
        return n;
    } else {
        const auto n { arduino::Serial.write(reinterpret_cast<uint8_t*>(ptr), len) };
        arduino::Serial.flush();
        return n;
    }
}

FLASHMEM uint8_t get_debug_led_pin() {
    return ctbot::CtBotConfig::DEBUG_LED_PIN;
}

FLASHMEM void serialport_put(const char c) {
    using namespace ctbot;

    CtBot& ctbot { CtBot::get_instance() };
    arduino::SerialIO* p_serial { ctbot.get_serial_cmd() };

    if (p_serial) {
        p_serial->write_direct(c);
    } else {
        arduino::Serial.write(c);
    }
}

FLASHMEM void serialport_puts(const char* str) {
    using namespace ctbot;

    CtBot& ctbot { CtBot::get_instance() };
    arduino::SerialIO* p_serial { ctbot.get_serial_cmd() };

    const auto length { std::strlen(str) };
    if (p_serial) {
        p_serial->write_direct(str);
    } else {
        arduino::Serial.write(reinterpret_cast<const uint8_t*>(str), length);
    }
}

FLASHMEM void serialport_flush() {
    using namespace ctbot;

    CtBot& ctbot { CtBot::get_instance() };
    arduino::SerialIO* p_serial { ctbot.get_serial_cmd() };

    if (p_serial) {
        p_serial->flush_direct();
    } else {
        arduino::Serial.flush();
    }
    freertos::delay_ms(100);
}
} // extern C


#ifdef CTBOT_SIMULATION
#include "cxxopts.hpp"
#include "sim_connection.h"

int main(int argc, char** argv) {
    cxxopts::Options options { argv[0], "ct-Bot Teensy framework" };
    options.allow_unrecognised_options();

    // clang-format off
    options.add_options()
        ("t,host", "Hostname of ct-Sim", cxxopts::value<std::string>()->default_value("localhost"), "HOSTNAME")
        ("p,port", "Port of ct-Sim", cxxopts::value<std::string>()->default_value("10001"), "PORT")
        ("h,help", "Print usage")
        ;
    // clang-format on
    auto result { options.parse(argc, argv) };

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    ::serialport_puts("ct-Bot Teensy framework starting...\r\n");

    ctbot::SimConnection sim_conn { result["host"].as<std::string>(), result["port"].as<std::string>() };
    Serial.begin(0);

    setup();

    return 0;
}
#endif // CTBOT_SIMULATION

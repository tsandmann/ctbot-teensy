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
 * @file    ctbot_main.cpp
 * @brief   Entry point of c't-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "ctbot.h"
#include "ctbot_config.h"
#include "timer.h"
#include "scheduler.h"
#include "serial_connection_teensy.h"
#include "tests.h"

#include "arduino_freertos.h"
#include "portable/teensy.h"

#include <thread>
#include <memory>


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

/**
 * @brief Task to initialize everything
 * @note This task is suspended forever after initialization is done.
 */
static FLASHMEM void init_task() {
    using namespace ctbot;

    /* wait for USB device enumeration, terminal program connection, etc. */
    Timer::delay_us(CtBotConfig::BOOT_DELAY_MS * 1'000UL);
    ::serial_puts(PSTR("\r\n"));

    // ::serial_puts(PSTR("init_task()"));
    // freertos::print_ram_usage();

    /* create CtBot singleton instance... */
    // ::serial_puts(PSTR("creating CtBot instance..."));
    CtBot& ctbot { CtBot::get_instance() };

    /* initialize it... */
    // ::serial_puts(PSTR("calling ctbot.setup()..."));
    ctbot.setup(true);
    ::serial_puts(PSTR("ctbot.setup() done."));

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
            ::serial_puts(PSTR("deleting g_button_test..."));
            delete g_button_test;
            ::serial_puts(PSTR("done."));
        });
    }

    if (CtBotConfig::TASKWAIT_TEST_AVAILABLE) {
        g_wait_test = new tests::TaskWaitTest { ctbot };
        std::atexit([]() { delete g_wait_test; });
    }

    freertos::print_ram_usage();

    ::vTaskPrioritySet(nullptr, tskIDLE_PRIORITY);
    // ::serial_puts(PSTR("deleting init task..."));
    ::vTaskDelete(nullptr);
}

extern "C" {
/**
 * @brief Entry point for c't-Bot initialization
 *
 * @startuml{main.png}
 *  activate Main
 *  Main -> CtBot: get_instance()
 *  Main <-- CtBot: ctbot
 *
 *  Main -> CtBot: setup()
 *  activate CtBot
 *  create Scheduler
 *  CtBot -> Scheduler: new
 *  CtBot -> Scheduler: task_add("main")
 *  activate Scheduler
 *  CtBot <-- Scheduler
 *  deactivate Scheduler
 *
 *  Main <-- CtBot
 *  deactivate CtBot
 *
 *  alt BLINK_TEST_AVAILABLE == true
 *   create BlinkTest
 *   Main -> BlinkTest: new
 *   BlinkTest -> Scheduler: task_add("blinktest")
 *   activate Scheduler
 *   BlinkTest <-- Scheduler
 *   deactivate Scheduler
 *   activate BlinkTest
 *   Main <-- BlinkTest
 *  end
 *
 *  alt LED_TEST_AVAILABLE == true
 *   create LedTest
 *   Main -> LedTest: new
 *   LedTest -> Scheduler: task_add("ledtest")
 *   activate Scheduler
 *   LedTest <-- Scheduler
 *   deactivate Scheduler
 *   activate LedTest
 *   Main <-- LedTest
 *  end
 *
 *  alt LCD_TEST_AVAILABLE == true
 *   create LcdTest
 *   Main -> LcdTest: new
 *   LcdTest -> Scheduler: task_add("lcdtest")
 *   activate Scheduler
 *   LcdTest <-- Scheduler
 *   deactivate Scheduler
 *   activate LcdTest
 *   Main <-- LcdTest
 *  end
 *
 *  alt ENA_TEST_AVAILABLE == true
 *   create EnaTest
 *   Main -> EnaTest: new
 *   EnaTest -> Scheduler: task_add("enatest")
 *   activate Scheduler
 *   EnaTest <-- Scheduler
 *   deactivate Scheduler
 *   activate EnaTest
 *   Main <-- EnaTest
 *  end
 *
 *  alt SENS_LCD_TEST_AVAILABLE == true
 *   create SensorLcdTest
 *   Main -> SensorLcdTest: new
 *   SensorLcdTest -> Scheduler: task_add("senstest")
 *   activate Scheduler
 *   SensorLcdTest <-- Scheduler
 *   deactivate Scheduler
 *   activate SensorLcdTest
 *   Main <-- SensorLcdTest
 *  end
 *
 *  Main -> CtBot: start()
 *  activate CtBot
 *  ... **run until shutdown** ...
 *  Main <-- CtBot
 *  deactivate SensorLcdTest
 *  destroy SensorLcdTest
 *  deactivate EnaTest
 *  destroy EnaTest
 *  deactivate LcdTest
 *  destroy LcdTest
 *  deactivate LedTest
 *  destroy LedTest
 *  deactivate BlinkTest
 *  destroy BlinkTest
 *  note over Scheduler: tasks aren't really destroyed on shutdown
 *  deactivate CtBot
 * @enduml
 */
void setup() {
    using namespace ctbot;
    arduino::pinMode(CtBotConfig::DEBUG_LED_PIN, arduino::OUTPUT);
    arduino::digitalWriteFast(CtBotConfig::DEBUG_LED_PIN, true); // turn debug LED on

    const auto last { free_rtos_std::gthr_freertos::set_next_stacksize(2048) };
    auto p_init_thread { std::make_unique<std::thread>([]() { init_task(); }) };
    free_rtos_std::gthr_freertos::set_name(p_init_thread.get(), PSTR("INIT"));
    free_rtos_std::gthr_freertos::set_next_stacksize(last);

    arduino::digitalWriteFast(CtBotConfig::DEBUG_LED_PIN, false); // turn debug LED off

    ::vTaskStartScheduler();

    freertos::error_blink(10);
    // we never ever get here
}

/**
 * @brief Arduino loop function, not used because of scheduler (@see Scheduler)
 */
void loop() {}

/**
 * @brief libc "syscall" implementation for writing characters to a filestream
 * @param[in] ptr: Pointer to data to write
 * @param[in] len: Number of bytes to write
 * @return Number of bytes written, -1 in case of an error
 * @note File handle paramter is ignored, any data is written to serial connection
 */
int _write(int, char* ptr, int len) {
    using namespace ctbot;

    static CtBot& ctbot { CtBot::get_instance() };
    SerialConnectionTeensy* p_serial { ctbot.get_serial_usb_conn() };

    if (p_serial) {
        const auto n { p_serial->send(reinterpret_cast<uint8_t*>(ptr), len) };
        p_serial->flush();
        return n;
    } else {
        const auto n { arduino::Serial.write(reinterpret_cast<uint8_t*>(ptr), len) };
        arduino::Serial.flush();
        return n;
    }
}

} // extern C

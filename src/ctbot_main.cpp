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
#include "scheduler.h"
#include "timer.h"
#include "serial_connection_teensy.h"
#include "ctbot_config.h"
#include "leds.h"
#include "tests.h"


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
    /* wait for USB device enumeration, terminal program connection, etc. */
    ctbot::Timer::delay(1500U);

    /* create CtBot singleton instance... */
    auto& ctbot { ctbot::CtBot::get_instance() };

    /* initialize it... */
    ctbot.setup();

    /* create test tasks if configured... */
    if (ctbot::CtBotConfig::BLINK_TEST_AVAILABLE) {
        new ctbot::tests::BlinkTest(ctbot);
    }

    if (ctbot::CtBotConfig::LED_TEST_AVAILABLE) {
        new ctbot::tests::LedTest(ctbot);
    }

    if (ctbot::CtBotConfig::LCD_TEST_AVAILABLE) {
        new ctbot::tests::LcdTest(ctbot);
    }

    if (ctbot::CtBotConfig::ENA_TEST_AVAILABLE) {
        new ctbot::tests::EnaTest(ctbot);
    }

    if (ctbot::CtBotConfig::SENS_LCD_TEST_AVAILABLE) {
        new ctbot::tests::SensorLcdTest(ctbot);
    }

    /* finally start CtBot instance */
    ctbot.start();

    // we should never get here
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
    static auto& ctbot { ctbot::CtBot::get_instance() };
    auto p_serial { ctbot.get_serial_conn() };

    if (p_serial) {
        return p_serial->send(reinterpret_cast<uint8_t*>(ptr), len);
    }
    return -1;
}

} // extern C

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

#include "arduino_fixed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portable/teensy.h"
#include <thread>
#include <memory>


namespace ctbot {
// FIXME: just for testing purpose
tests::TaskWaitTest* p_wait_test { nullptr };
} // namespace ctbot

static TaskHandle_t g_audio_task { nullptr };

/**
 * @brief Task to initialize everything
 * @note This task is suspended forever after initialization is done.
 */
static void init_task() {
    using namespace ctbot;

    /* wait for USB device enumeration, terminal program connection, etc. */
    Timer::delay_us(1500UL * 1000UL);

    // serial_puts("init_task()");
    // freertos::print_ram_usage();

    /* create CtBot singleton instance... */
    // serial_puts("creating CtBot instance...");
    CtBot& ctbot { CtBot::get_instance() };

    /* initialize it... */
    // serial_puts("calling ctbot.setup()...");
    ctbot.setup();
    // serial_puts("ctbot.setup() done.");

    /* create test tasks if configured... */
    if (ctbot::CtBotConfig::BLINK_TEST_AVAILABLE) {
        new tests::BlinkTest(ctbot);
    }

    if (ctbot::CtBotConfig::LED_TEST_AVAILABLE) {
        new tests::LedTest(ctbot);
    }

    if (ctbot::CtBotConfig::LCD_TEST_AVAILABLE) {
        new tests::LcdTest(ctbot);
    }

    if (ctbot::CtBotConfig::ENA_TEST_AVAILABLE) {
        new tests::EnaTest(ctbot);
    }

    if (ctbot::CtBotConfig::SENS_LCD_TEST_AVAILABLE) {
        new tests::SensorLcdTest(ctbot);
    }

    if (ctbot::CtBotConfig::TASKWAIT_TEST_AVAILABLE) {
        ctbot::p_wait_test = new tests::TaskWaitTest(ctbot);
    }

    // extern unsigned long __bss_end__; // set by linker script
    // extern unsigned long _estack; // set by linker script
    // Serial.print("__bss_end__=0x");
    // Serial.println(reinterpret_cast<uintptr_t>(&__bss_end__), 16);
    // Serial.print("_estack=0x");
    // Serial.println(reinterpret_cast<uintptr_t>(&_estack), 16);
    freertos::print_ram_usage();


    if (ctbot::CtBotConfig::AUDIO_AVAILABLE) {
        ctbot.get_scheduler()->task_add("audio", 1, Scheduler::MAX_PRIORITY, 512, []() {
            while (true) {
                ::software_isr(); // AudioStream::update_all()
                ::xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
            }
        });
        g_audio_task = ::xTaskGetHandle("audio");
    }

    ::vTaskPrioritySet(nullptr, tskIDLE_PRIORITY);
    // serial_puts("deleting init task...");
    ::vTaskDelete(nullptr);
}

extern "C" {
void softirq_isr();

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
    __disable_irq();

    _VectorsRam[80] = softirq_isr;

    freertos::sysview_init();

    // delay_us(2000UL * 1000UL);

    // serial_puts("\n\nCreating init task...");
    const auto last { free_rtos_std::gthr_freertos::set_next_stacksize(2048) };
    auto p_init_thread { std::make_unique<std::thread>([]() { init_task(); }) };
    free_rtos_std::gthr_freertos::set_name(p_init_thread.get(), "INIT");
    free_rtos_std::gthr_freertos::set_next_stacksize(last);

    ::vTaskStartScheduler();

    // we never ever get here
    if (p_init_thread->joinable()) {
        p_init_thread->join();
    }
    freertos::error_blink(10);
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
        return p_serial->send(reinterpret_cast<uint8_t*>(ptr), len);
    }
    return -1;
}


/**
 * @brief Hard fault - blink four short flash every two seconds
 */
void hard_fault_isr() {
    freertos::error_blink(4);
}

/**
 * @brief Bus fault - blink five short flashes every two seconds
 */
void bus_fault_isr() {
    freertos::error_blink(5);
}

/**
 * @brief Usage fault - blink six short flashes every two seconds
 */
void usage_fault_isr() {
    freertos::error_blink(6);
}

void softirq_isr() {
    if (ctbot::CtBotConfig::AUDIO_AVAILABLE) {
        freertos::trace_isr_enter();
        if (g_audio_task) {
            ::xTaskNotifyFromISR(g_audio_task, 0, eNoAction, nullptr);
            portYIELD_FROM_ISR(true);
        }
        freertos::trace_isr_exit();
    }
}

} // extern C

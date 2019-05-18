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

#include "kinetis.h"
#include "arduino_fixed.h"


/**
 * @brief Task to initialize everything
 * @note This task is suspended forever after initialization is done.
 */
static void init_task(void*) {
    using namespace ctbot;

    /* wait for USB device enumeration, terminal program connection, etc. */
    Timer::delay_us(2'000UL * 1'000UL);

    // ::serial_puts("init_task()");

    /* create CtBot singleton instance... */
    // ::serial_puts("creating CtBot instance...");
    CtBot& ctbot { CtBot::get_instance() };

    /* initialize it... */
    // ::serial_puts("calling ctbot.setup()...");
    ctbot.setup();
    // ::serial_puts("ctbot.setup() done.");

    /* create test tasks if configured... */
    if (CtBotConfig::BLINK_TEST_AVAILABLE) {
        new tests::BlinkTest(ctbot);
    }

    if (CtBotConfig::LED_TEST_AVAILABLE) {
        new tests::LedTest(ctbot);
    }

    if (CtBotConfig::LCD_TEST_AVAILABLE) {
        new tests::LcdTest(ctbot);
    }

    if (CtBotConfig::ENA_TEST_AVAILABLE) {
        new tests::EnaTest(ctbot);
    }

    if (CtBotConfig::SENS_LCD_TEST_AVAILABLE) {
        new tests::SensorLcdTest(ctbot);
    }

    /* finally start CtBot instance */
    // arduino::Serial.print("starting CtBot instance... ");
    ctbot.start();
    // arduino::Serial.println("CtBot instance exited.");

    // we should never get here
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
    init_task(nullptr);
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

int _kill(int, int) {
    while (true) {
    }
}

int _getpid() {
    return 1;
}

#ifndef MAIN_STACK_SIZE
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define MAIN_STACK_SIZE 8192UL
#else
#error "Unknown architecture"
#endif
#endif // MAIN_STACK_SIZE

asm(".global _printf_float"); /**< to have a printf supporting floating point values */

/* override _sbrk() - you have to link with option: "-Wl,--wrap=_sbrk" */
extern unsigned long __bss_end__; // set by linker script
extern unsigned long _estack; // set by linker script
void* __wrap__sbrk(ptrdiff_t incr) {
    static uint8_t* currentHeapEnd { reinterpret_cast<uint8_t*>(&__bss_end__) };

    void* previousHeapEnd = currentHeapEnd;

    if ((reinterpret_cast<uintptr_t>(currentHeapEnd) + incr >= reinterpret_cast<uintptr_t>(&_estack) - MAIN_STACK_SIZE)
        || (reinterpret_cast<uintptr_t>(currentHeapEnd) + incr < reinterpret_cast<uintptr_t>(&__bss_end__))) {
        _impure_ptr->_errno = ENOMEM; // newlib's thread-specific errno
        return reinterpret_cast<void*>(-1); // the malloc-family routine that called sbrk will return 0
    }

    currentHeapEnd += incr;
    return previousHeapEnd;
}

/**
 * @brief Check for USB events pending and call the USB ISR
 */
static void poll_usb() {
    if (SIM_SCGC4 & SIM_SCGC4_USBOTG) {
        ::usb_isr();
    }
}

/**
 * @brief Delay between led error flashes
 * @param[in] ms: Milliseconds to delay
 * @note Doesn't use a timer to work with interrupts disabled
 */
static void delay_ms(const uint32_t ms) {
    const uint32_t n { ms / 10 };
    for (uint32_t i { 0 }; i < n; ++i) {
        poll_usb();

        for (uint32_t i { 0 }; i < 10UL * (F_CPU / 7000UL); ++i) { // 10 ms
            asm volatile("nop");
        }
    }

    const uint32_t iterations { (ms % 10) * (F_CPU / 7000UL) }; // remainder
    for (uint32_t i { 0 }; i < iterations; ++i) {
        asm volatile("nop");
    }
}

static void error_blink(const uint8_t n) {
    __disable_irq();
    arduino::digitalWriteFast(35, false); // disable pwm output
    arduino::pinMode(35, arduino::OUTPUT);
    arduino::digitalWriteFast(36, false); // disable pwm output
    arduino::pinMode(36, arduino::OUTPUT);
    arduino::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);

    while (true) {
        for (uint8_t i { 0 }; i < n; ++i) {
            arduino::digitalWriteFast(arduino::LED_BUILTIN, true);
            delay_ms(300UL);
            arduino::digitalWriteFast(arduino::LED_BUILTIN, false);
            delay_ms(300UL);
        }
        delay_ms(2'000UL);
    }
}

/**
 * @brief Hard fault - blink four short flash every two seconds
 */
void hard_fault_isr() {
    error_blink(4);
}

/**
 * @brief Bus fault - blink five short flashes every two seconds
 */
void bus_fault_isr() {
    error_blink(5);
}

/**
 * @brief Usage fault - blink six short flashes every two seconds
 */
void usage_fault_isr() {
    error_blink(6);
}

} // extern C

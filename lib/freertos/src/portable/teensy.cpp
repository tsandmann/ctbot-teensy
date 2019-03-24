/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    teensy.cpp
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 3
 * @author  Timo Sandmann
 * @date    26.05.2018
 */

#include "teensy.h"


/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers. That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "private/portable.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "Arduino.h"
#include "kinetis.h"
#include "util/atomic.h"

#include <new>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <malloc.h>
#include <cstring>


#ifndef MAIN_STACK_SIZE
#if defined(__MKL26Z64__)
#define MAIN_STACK_SIZE 512UL
#elif defined(__MK20DX128__)
#define MAIN_STACK_SIZE 1024UL
#elif defined(__MK20DX256__)
#define MAIN_STACK_SIZE 1024UL
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define MAIN_STACK_SIZE 1024UL
#endif
#else
#error "Unknown architecture"
#endif // MAIN_STACK_SIZE

extern "C" {
asm(".global _printf_float"); /**< to have a printf supporting floating point values */

extern unsigned long __bss_end__; // set by linker script
extern unsigned long _estack; // set by linker script
static UBaseType_t int_nesting { 0 }; // used by __malloc_lock()
static uint32_t int_prio { 0 }; // used by __malloc_lock()

void serial_puts(const char* str) {
    Serial.println(str);
    Serial.flush();
}

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
void assert_blink(const char* file, int line, const char* func, const char* expr) {
    Serial.print("ASSERT in [");
    Serial.print(file);
    Serial.print(':');
    Serial.print(line, 10);
    Serial.print("] ");
    Serial.print(func);
    Serial.print(": ");
    Serial.println(expr);
    Serial.flush();

    freertos::error_blink(1);
}
} // extern C


void* operator new(size_t n, const std::nothrow_t&) noexcept {
    return malloc(n);
}

void* operator new[](size_t n, const std::nothrow_t&) noexcept {
    return malloc(n);
}

void operator delete(void* p, const std::nothrow_t&) noexcept {
    free(p);
}

void operator delete[](void* p, const std::nothrow_t&) noexcept {
    free(p);
}


namespace freertos {
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

void error_blink(const uint8_t n) {
    __disable_irq();
    ::digitalWriteFast(35, false); // disable pwm output
    ::pinMode(35, OUTPUT);
    ::digitalWriteFast(36, false); // disable pwm output
    ::pinMode(36, OUTPUT);
    ::pinMode(LED_BUILTIN, OUTPUT);

    while (true) {
        for (uint8_t i { 0 }; i < n; ++i) {
            ::digitalWriteFast(LED_BUILTIN, true);
            delay_ms(300UL);
            ::digitalWriteFast(LED_BUILTIN, false);
            delay_ms(300UL);
        }
        delay_ms(2000UL);
    }
}

std::tuple<size_t, size_t, size_t> ram_usage() {
    volatile size_t x { 0 };
    void* ptr { sbrk(x) };
    const size_t system_free { (reinterpret_cast<uint8_t*>(&_estack) - static_cast<uint8_t*>(ptr)) - MAIN_STACK_SIZE };
    const auto info { mallinfo() };
    const std::tuple<size_t, size_t, size_t> ret { system_free + info.fordblks, info.uordblks, system_free };
    return ret;
}

void print_ram_usage() {
    const auto info { ram_usage() };

    Serial.print("free RAM: ");
    Serial.print(std::get<0>(info) / 1024UL, 10);
    Serial.print(" KB, used heap: ");
    Serial.print(std::get<1>(info) / 1024UL, 10);
    Serial.print(" KB, system free: ");
    Serial.print(std::get<2>(info) / 1024UL, 10);
    Serial.println(" KB");
}

uint64_t get_us() {
    uint32_t current, load, count, istatus;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current = SYST_CVR;
        count = get_ms();
        istatus = SCB_ICSR; // bit 26 indicates if systick exception pending
        load = SYST_RVR;
    }

    if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) {
        ++count;
    }

    current = load - current;
#if (configUSE_TICKLESS_IDLE == 1)
#warning "tickless idle mode is untested"
    return static_cast<uint64_t>(count) * 1000U + current * 1000U / (load + 1U);
#else
    return static_cast<uint64_t>(count) * 1000U + current / (configCPU_CLOCK_HZ / configTICK_RATE_HZ / 1000U);
#endif
}

void sysview_init() {
    SEGGER_SYSVIEW_Conf();
}
} // namespace freertos


extern "C" {
// override _sbrk() - you have to link with option: "-Wl,--wrap=_sbrk"
void* _sbrk(ptrdiff_t);
void* __wrap__sbrk(ptrdiff_t incr) {
    static uint8_t* currentHeapEnd { reinterpret_cast<uint8_t*>(&__bss_end__) };

    static_assert(portSTACK_GROWTH == -1, "Stack growth down assumed");

    // Serial.print("_sbrk(");
    // Serial.print(incr, 10);
    // Serial.println(")");

    void* previousHeapEnd = currentHeapEnd;

    if ((reinterpret_cast<uintptr_t>(currentHeapEnd) + incr >= reinterpret_cast<uintptr_t>(&_estack) - MAIN_STACK_SIZE)
        || (reinterpret_cast<uintptr_t>(currentHeapEnd) + incr < reinterpret_cast<uintptr_t>(&__bss_end__))) {
#if (configUSE_MALLOC_FAILED_HOOK == 1)
        {
            extern void vApplicationMallocFailedHook();
            vApplicationMallocFailedHook();
        }
#else
        // If you prefer to believe your application will gracefully trap out-of-memory...
        _impure_ptr->_errno = ENOMEM; // newlib's thread-specific errno
#endif
        return reinterpret_cast<void*>(-1); // the malloc-family routine that called sbrk will return 0
    }

    currentHeapEnd += incr;
    // Serial.print("currentHeapEnd=0x");
    // Serial.print(reinterpret_cast<uintptr_t>(currentHeapEnd), 16);
    // Serial.print(" previousHeapEnd=0x");
    // Serial.println(reinterpret_cast<uintptr_t>(previousHeapEnd), 16);

    return previousHeapEnd;
}

void __malloc_lock(struct _reent*) {
    int_prio = ::ulPortRaiseBASEPRI();
    ++int_nesting;
}

void __malloc_unlock(struct _reent*) {
    // configASSERT(int_nesting);
    --int_nesting;
    if (!int_nesting) {
        ::vPortSetBASEPRI(int_prio);
    }
}

void* sbrk(ptrdiff_t incr) {
    const auto pri { ::ulPortRaiseBASEPRI() };
    const auto ptr { _sbrk(incr) };
    ::vPortSetBASEPRI(pri);
    return ptr;
}

int __wrap__gettimeofday(timeval* tv, void*) {
    const auto now_us { freertos::get_us() };
    *tv = timeval { static_cast<time_t>(now_us / 1'000'000U), static_cast<suseconds_t>(now_us % 1'000'000U) };
    return 0;
}

void vApplicationMallocFailedHook();
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName);

void vApplicationMallocFailedHook() {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap. pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores. */

    freertos::error_blink(2);
}

void vApplicationStackOverflowHook(TaskHandle_t, char* task_name) {
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.
    This hook function is called if a stack overflow is detected. */

    static char taskname[configMAX_TASK_NAME_LEN + 1];
    std::memcpy(taskname, task_name, configMAX_TASK_NAME_LEN);
    serial_puts("STACK OVERFLOW: ");
    serial_puts(taskname);

    freertos::error_blink(3);
}


#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook();
void vApplicationIdleHook() {
    ::yield();
}
#endif // configUSE_IDLE_HOOK

#if (configUSE_TICK_HOOK > 0)
extern "C" volatile uint32_t systick_millis_count;
void vApplicationTickHook();
void vApplicationTickHook() {
    systick_millis_count++;
}
#endif // configUSE_TICK_HOOK

#if (configSUPPORT_STATIC_ALLOCATION == 1)
/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize) {
    /* If the buffers to be provided to the Timer task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif // configSUPPORT_STATIC_ALLOCATION

} // extern C

/*
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
 * @file    arduino_fixed.h
 * @brief   Wrapper aroung Arduino stuff to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include "Wire.h"
#include "WString.h"
#include "Print.h"
#include "Stream.h"

#include <cstdint>
#include <atomic>
#include <deque>


class StdinOutWrapper : public Stream {
protected:
    std::atomic<bool> recv_running_;
    std::deque<char> in_buffer_;
    void* in_mutex_;

    static void receiver_task(void* p_instance);

public:
    StdinOutWrapper();
    ~StdinOutWrapper();

    virtual int available() override;
    virtual int peek() override;
    virtual int read() override;
    virtual size_t write(const uint8_t) override;
    virtual size_t write(const uint8_t*, size_t) override;
    virtual int availableForWrite() override;
    virtual void flush() override;

protected:
    bool key_pressed(uint32_t timeout_ms = 0);
};

class HardwareSerial : public Stream {
public:
    virtual int available() override {
        return 0;
    }
    virtual int peek() override {
        return -1;
    }
    virtual int read() override {
        return -1;
    }
    virtual size_t write(const uint8_t) override {
        return 0;
    }
    virtual size_t write(const uint8_t*, size_t) override {
        return 0;
    }
};

extern StdinOutWrapper Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
extern HardwareSerial Serial6;

namespace arduino {
int analogRead(uint8_t pin);
void analogReadAveraging(unsigned int num);
void analogReadResolution(unsigned int bits);
void analogWrite(uint8_t pin, int val);
void analogWriteFrequency(uint8_t pin, float frequency);
uint32_t analogWriteResolution(uint32_t bits);

void attachInterrupt(uint8_t pin, void (*function)(void), int mode);
void detachInterrupt(uint8_t pin);

uint8_t digitalReadFast(uint8_t pin);
void digitalWriteFast(uint8_t pin, uint8_t val);
void pinMode(uint8_t pin, uint8_t mode);

static constexpr uint8_t INPUT { 0 };
static constexpr uint8_t OUTPUT { 1 };
static constexpr uint8_t INPUT_PULLUP { 2 };
static constexpr uint8_t INPUT_PULLDOWN { 3 };
static constexpr uint8_t OUTPUT_OPENDRAIN { 4 };
static constexpr uint8_t INPUT_DISABLE { 5 };

static constexpr uint8_t LED_BUILTIN { 13 };

static constexpr uint8_t FALLING { 2 };
static constexpr uint8_t RISING { 3 };
static constexpr uint8_t CHANGE { 4 };

static constexpr uint8_t BUILTIN_SDCARD { 254 };

static constexpr bool digitalPinHasPWM(uint8_t p) {
    return (((p) >= 2 && (p) <= 10) || (p) == 14 || ((p) >= 20 && (p) <= 23) || (p) == 29 || (p) == 30 || ((p) >= 35 && (p) <= 38));
}

uint32_t micros();
uint32_t millis();

void delayMicroseconds(const uint32_t us);

using ::HardwareSerial;
using ::Print;
using ::Serial;
using ::Serial1;
using ::Serial2;
using ::Serial3;
using ::Serial4;
using ::Serial5;
using ::Serial6;
using ::Stream;
using ::String;
using ::Wire;
using ::Wire1;
using ::Wire2;
} // namespace arduino

static inline void __disable_irq() {}
static inline void __enable_irq() {}

void software_isr(void);

extern "C" {
extern void (*_VectorsRam[116])(void);
} // extern C

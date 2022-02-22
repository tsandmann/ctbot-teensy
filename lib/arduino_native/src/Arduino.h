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
 * @file    Arduino.h
 * @brief   Wrapper aroung Arduino stuff to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#pragma once

#include "Print.h"
#include "SPI.h"
#include "Stream.h"
#include "Wire.h"
#include "WString.h"
#include "serial_posix.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <pthread.h>


class StdinOutWrapper : public Stream {
protected:
    std::atomic<bool> recv_running_;
    pthread_t p_in_thread_;
    pthread_mutex_t in_buffer_mutex_;
    std::deque<char> in_buffer_;

public:
    StdinOutWrapper();
    ~StdinOutWrapper();

    virtual void begin(uint32_t);
    virtual void end();

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
    void addMemoryForRead(void*, size_t) {}
    void addMemoryForWrite(void*, size_t) {}
};

extern StdinOutWrapper Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
extern HardwareSerial Serial6;
extern HardwareSerial Serial7;
extern HardwareSerial Serial8;

extern "C" {
extern void (*_VectorsRam[255 + 16])(void);

static inline void NVIC_ENABLE_IRQ(uint8_t) {}
static inline void NVIC_DISABLE_IRQ(uint8_t) {}
static inline void NVIC_SET_PRIORITY(uint8_t, uint8_t) {}
} // extern C

namespace arduino {
int analogRead(uint8_t pin);
void analogReadAveraging(unsigned int num);
void analogReadResolution(unsigned int bits);
void analogWrite(uint8_t pin, int val);
void analogWriteFrequency(uint8_t pin, float frequency);
uint32_t analogWriteResolution(uint32_t bits);
void analogReference(uint8_t type);

void attachInterrupt(uint8_t pin, void (*function)(void), int mode);
void detachInterrupt(uint8_t pin);

uint8_t digitalReadFast(uint8_t pin);
void digitalWriteFast(uint8_t pin, uint8_t val);
void pinMode(uint8_t pin, uint8_t mode);

void attachInterruptVector(uint8_t irq, void (*function)(void));

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

static constexpr uint8_t IRQ_SOFTWARE { 70 };

static constexpr bool digitalPinHasPWM(uint8_t p) {
    return (((p) >= 2 && (p) <= 10) || (p) == 14 || ((p) >= 20 && (p) <= 23) || (p) == 29 || (p) == 30 || ((p) >= 35 && (p) <= 38));
}

template <typename T, typename std::enable_if_t<std::is_integral<T>::value, int> = 0>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T, typename std::enable_if_t<std::is_reference<T>::value, int> = 0,
    typename std::enable_if_t<std::is_integral<typename std::remove_reference<T>::type>::value, int> = 0>
T map(const T x, const T in_min, const T in_max, const T out_min, const T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
using ::Serial7;
using ::Serial8;
using ::SPI;
using ::SPI1;
using ::SPI2;
using ::Stream;
using ::String;
using ::TwoWire;
using ::Wire;
using ::Wire1;
using ::Wire2;
using ::Wire3;
} // namespace arduino

static inline void __disable_irq() {}
static inline void __enable_irq() {}

#define FILE_USE_MOVE

typedef struct {
    uint8_t sec; // 0-59
    uint8_t min; // 0-59
    uint8_t hour; // 0-23
    uint8_t wday; // 0-6, 0=sunday
    uint8_t mday; // 1-31
    uint8_t mon; // 0-11
    uint8_t year; // 70-206, 70=1970, 206=2106
} DateTimeFields;

class elapsedMillis {
private:
    unsigned long ms;

public:
    elapsedMillis(void) {
        ms = arduino::millis();
    }
    elapsedMillis(unsigned long val) {
        ms = arduino::millis() - val;
    }
    elapsedMillis(const elapsedMillis& orig) {
        ms = orig.ms;
    }
    operator unsigned long() const {
        return arduino::millis() - ms;
    }
    elapsedMillis& operator=(const elapsedMillis& rhs) {
        ms = rhs.ms;
        return *this;
    }
    elapsedMillis& operator=(unsigned long val) {
        ms = arduino::millis() - val;
        return *this;
    }
    elapsedMillis& operator-=(unsigned long val) {
        ms += val;
        return *this;
    }
    elapsedMillis& operator+=(unsigned long val) {
        ms -= val;
        return *this;
    }
    elapsedMillis operator-(int val) const {
        elapsedMillis r(*this);
        r.ms += val;
        return r;
    }
    elapsedMillis operator-(unsigned int val) const {
        elapsedMillis r(*this);
        r.ms += val;
        return r;
    }
    elapsedMillis operator-(long val) const {
        elapsedMillis r(*this);
        r.ms += val;
        return r;
    }
    elapsedMillis operator-(unsigned long val) const {
        elapsedMillis r(*this);
        r.ms += val;
        return r;
    }
    elapsedMillis operator+(int val) const {
        elapsedMillis r(*this);
        r.ms -= val;
        return r;
    }
    elapsedMillis operator+(unsigned int val) const {
        elapsedMillis r(*this);
        r.ms -= val;
        return r;
    }
    elapsedMillis operator+(long val) const {
        elapsedMillis r(*this);
        r.ms -= val;
        return r;
    }
    elapsedMillis operator+(unsigned long val) const {
        elapsedMillis r(*this);
        r.ms -= val;
        return r;
    }
};

class elapsedMicros {
private:
    unsigned long us;

public:
    elapsedMicros(void) {
        us = arduino::micros();
    }
    elapsedMicros(unsigned long val) {
        us = arduino::micros() - val;
    }
    elapsedMicros(const elapsedMicros& orig) {
        us = orig.us;
    }
    operator unsigned long() const {
        return arduino::micros() - us;
    }
    elapsedMicros& operator=(const elapsedMicros& rhs) {
        us = rhs.us;
        return *this;
    }
    elapsedMicros& operator=(unsigned long val) {
        us = arduino::micros() - val;
        return *this;
    }
    elapsedMicros& operator-=(unsigned long val) {
        us += val;
        return *this;
    }
    elapsedMicros& operator+=(unsigned long val) {
        us -= val;
        return *this;
    }
    elapsedMicros operator-(int val) const {
        elapsedMicros r(*this);
        r.us += val;
        return r;
    }
    elapsedMicros operator-(unsigned int val) const {
        elapsedMicros r(*this);
        r.us += val;
        return r;
    }
    elapsedMicros operator-(long val) const {
        elapsedMicros r(*this);
        r.us += val;
        return r;
    }
    elapsedMicros operator-(unsigned long val) const {
        elapsedMicros r(*this);
        r.us += val;
        return r;
    }
    elapsedMicros operator+(int val) const {
        elapsedMicros r(*this);
        r.us -= val;
        return r;
    }
    elapsedMicros operator+(unsigned int val) const {
        elapsedMicros r(*this);
        r.us -= val;
        return r;
    }
    elapsedMicros operator+(long val) const {
        elapsedMicros r(*this);
        r.us -= val;
        return r;
    }
    elapsedMicros operator+(unsigned long val) const {
        elapsedMicros r(*this);
        r.us -= val;
        return r;
    }
};

class CrashReportClass : public Printable {
public:
    virtual size_t printTo(Print&) const {
        return 0;
    }
    void clear() {}
    operator bool() {
        return false;
    }
};

extern CrashReportClass CrashReport;

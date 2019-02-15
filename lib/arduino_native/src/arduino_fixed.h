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

#include <Wire.h>
#include <cstdint>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>


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

class __FlashStringHelper;
using String = std::string;

// FIXME: to be implemented
class Print {
public:
    constexpr Print() {}
    virtual size_t write(uint8_t) {
        return 0;
    }
    size_t write(const char* str) {
        return write((const uint8_t*) str, strlen(str));
    }
    virtual size_t write(const uint8_t*, size_t) {
        return 0;
    }
    virtual int availableForWrite(void) {
        return 0;
    }
    virtual void flush() {}
    size_t write(const char* buffer, size_t size) {
        return write((const uint8_t*) buffer, size);
    }
    size_t print(const String&) {
        return 0;
    }
    size_t print(char c) {
        return write((uint8_t) c);
    }
    size_t print(const char s[]) {
        return write(s);
    }
    size_t print(const __FlashStringHelper* f) {
        return write((const char*) f);
    }

    // size_t print(uint8_t b) {
    //     return printNumber(b, 10, 0);
    // }
    // size_t print(int n) {
    //     return print((long) n);
    // }
    // size_t print(unsigned int n) {
    //     return printNumber(n, 10, 0);
    // }
    // size_t print(long n);
    // size_t print(unsigned long n) {
    //     return printNumber(n, 10, 0);
    // }

    // size_t print(unsigned char n, int base) {
    //     return printNumber(n, base, 0);
    // }
    // size_t print(int n, int base) {
    //     return (base == 10) ? print(n) : printNumber(n, base, 0);
    // }
    // size_t print(unsigned int n, int base) {
    //     return printNumber(n, base, 0);
    // }
    // size_t print(long n, int base) {
    //     return (base == 10) ? print(n) : printNumber(n, base, 0);
    // }
    // size_t print(unsigned long n, int base) {
    //     return printNumber(n, base, 0);
    // }

    // size_t print(double n, int digits = 2) {
    //     return printFloat(n, digits);
    // }
    // size_t print(const Printable& obj) {
    //     return obj.printTo(*this);
    // }
    size_t println() {
        return 0;
    }
    size_t println(const String& s) {
        return print(s) + println();
    }
    size_t println(char c) {
        return print(c) + println();
    }
    size_t println(const char s[]) {
        return print(s) + println();
    }
    size_t println(const __FlashStringHelper* f) {
        return print(f) + println();
    }

    size_t println(uint8_t b) {
        return print(b) + println();
    }
    size_t println(int n) {
        return print(n) + println();
    }
    size_t println(unsigned int n) {
        return print(n) + println();
    }
    size_t println(long n) {
        return print(n) + println();
    }
    size_t println(unsigned long n) {
        return print(n) + println();
    }

    // size_t println(unsigned char n, int base) {
    //     return print(n, base) + println();
    // }
    // size_t println(int n, int base) {
    //     return print(n, base) + println();
    // }
    // size_t println(unsigned int n, int base) {
    //     return print(n, base) + println();
    // }
    // size_t println(long n, int base) {
    //     return print(n, base) + println();
    // }
    // size_t println(unsigned long n, int base) {
    //     return print(n, base) + println();
    // }

    // size_t println(double n, int digits = 2) {
    //     return print(n, digits) + println();
    // }
    // size_t println(const Printable& obj) {
    //     return obj.printTo(*this) + println();
    // }
    int getWriteError() {
        return 0;
    }
    void clearWriteError() {}
    int printf(const char* format, ...);
    int printf(const __FlashStringHelper* format, ...);
};

class Stream : public Print {
protected:
    std::thread* p_recv_thread_;
    std::atomic<bool> recv_running_;
    std::deque<char> in_buffer_;
    std::mutex in_mutex_;

public:
    Stream();
    void setRX(uint8_t pin);
    void setTX(uint8_t pin);
    void begin(uint32_t baud);

    int available();
    int peek();
    int read();
    uint32_t readBytes(void* buffer, size_t length);
    uint32_t write(const char data);
    uint32_t write(const void* buffer, size_t length);
    virtual void flush() override;
    virtual size_t write(uint8_t) override {
        return 0;
    }
};

class StdinWrapper : public Stream {
public:
    StdinWrapper();

    ~StdinWrapper();

protected:
    bool key_pressed(uint32_t timeout_ms = 0);
};

class HardwareSerial : public Stream {};

extern StdinWrapper Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
extern HardwareSerial Serial6;
} // namespace arduino

static inline void __disable_irq() {}
static inline void __enable_irq() {}

void software_isr(void);

extern "C" {
extern void (*_VectorsRam[116])(void);
} // extern C

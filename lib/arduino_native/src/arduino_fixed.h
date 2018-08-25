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

#ifndef _ARDUINO_FIXED_H_
#define _ARDUINO_FIXED_H_

#include <Wire.h>
#include <cstdint>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>


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

static constexpr bool digitalPinHasPWM(uint8_t p) {
    return (((p) >= 2 && (p) <= 10) || (p) == 14 || ((p) >= 20 && (p) <= 23) || (p) == 29 || (p) == 30 || ((p) >= 35 && (p) <= 38));
}

uint32_t micros();
uint32_t millis();

void delayMicroseconds(const uint32_t us);

class Stream {
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
    void flush();
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

// FIXME: this is just a temporary workaround
static constexpr uint32_t SYST_CVR { 0 };
static constexpr uint32_t SYST_RVR { 0 };
static constexpr uint32_t SCB_ICSR { 0 };
static constexpr uint32_t SCB_ICSR_PENDSTSET { 0 };

#endif /* _ARDUINO_FIXED_H_ */

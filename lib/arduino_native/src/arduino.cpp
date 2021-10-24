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
 * @file    arduino.cpp
 * @brief   Wrapper aroung Arduino stuff to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    10.06.2018
 */

#include "arduino_freertos.h"
#include "Wire.h"
#include "SPI.h"
#include "serial_posix.h"

#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <thread>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <termios.h>


static struct termios t_old;

namespace arduino {
static auto g_start_time { std::chrono::high_resolution_clock::now() };
static std::vector<bool> g_digital_pins(256);
static std::vector<int16_t> g_analog_pins(256);

KINETIS_I2C_t i2c_dummy;

uint32_t micros() __attribute__((weak));
uint32_t micros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - g_start_time).count();
}

uint32_t millis() __attribute__((weak));
uint32_t millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - g_start_time).count();
}

void delayMicroseconds(const uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

int analogRead(uint8_t pin) {
    try {
        return g_analog_pins.at(pin);
    } catch (const std::exception& e) {
        std::cerr << "arduino::analogRead(" << static_cast<uint16_t>(pin) << "): " << e.what() << "\n";
        return -1;
    }
}

void analogReadAveraging(unsigned int) {}

void analogReadResolution(unsigned int) {}

void analogReference(uint8_t) {}

void analogWrite(uint8_t pin, int val) {
    try {
        g_analog_pins.at(pin) = val;
    } catch (const std::exception& e) {
        std::cerr << "arduino::analogWrite(" << static_cast<uint16_t>(pin) << ", " << val << "): " << e.what() << "\n";
    }
}

void analogWriteFrequency(uint8_t, float) {}

uint32_t analogWriteResolution(uint32_t bits) {
    static uint32_t res { 0 };
    const uint32_t old { res };
    res = bits;
    return old;
}

void attachInterrupt(uint8_t, void (*)(void), int) {}

void detachInterrupt(uint8_t) {}

uint8_t digitalReadFast(uint8_t pin) {
    try {
        return g_digital_pins.at(pin);
    } catch (const std::exception& e) {
        std::cerr << "arduino::digitalReadFast(" << static_cast<uint16_t>(pin) << "): " << e.what() << "\n";
        return 0xff;
    }
}

void digitalWriteFast(uint8_t pin, uint8_t val) {
    try {
        g_digital_pins.at(pin) = val;
    } catch (const std::exception& e) {
        std::cerr << "arduino::digitalWriteFast(" << static_cast<uint16_t>(pin) << ", " << static_cast<uint16_t>(val) << "): " << e.what() << "\n";
    }
}

void pinMode(uint8_t, uint8_t) {}

void attachInterruptVector(uint8_t irq, void (*function)(void)) {
    _VectorsRam[irq + 16] = function;
}
} // namespace arduino

void software_isr() {}

StdinOutWrapper::StdinOutWrapper() : recv_running_ {}, p_in_thread_ { 0 } {}

StdinOutWrapper::~StdinOutWrapper() {
    end();
}

void StdinOutWrapper::begin(uint32_t) {
    // std::cout << "StdinOutWrapper::begin()\n";
    pthread_mutex_init(&in_buffer_mutex_, nullptr);
    recv_running_ = true;

    pthread_create(
        &p_in_thread_, nullptr,
        [](void* ptr) -> void* {
            StdinOutWrapper* p_this { reinterpret_cast<StdinOutWrapper*>(ptr) };

            sigset_t set;
            sigfillset(&set);
            pthread_sigmask(SIG_SETMASK, &set, nullptr);

            signal(SIGINT, [](int) {
                tcsetattr(1, TCSANOW, &t_old);
                _exit(1);
            });

            struct termios t;
            tcgetattr(1, &t);
            t_old = t;
            t.c_lflag &= (~ECHO & ~ICANON);
            t.c_cc[VMIN] = 0;
            t.c_cc[VTIME] = 1;
            tcsetattr(1, TCSANOW, &t);
            std::atexit([]() { tcsetattr(1, TCSANOW, &t_old); });

            while (p_this->recv_running_) {
                char buf;
                if (::read(1, &buf, 1) > 0) {
                    const char c { static_cast<char>(buf) };
                    pthread_mutex_lock(&p_this->in_buffer_mutex_);
                    p_this->in_buffer_.push_back(c);
                    pthread_mutex_unlock(&p_this->in_buffer_mutex_);
                }
            }

            tcsetattr(1, TCSANOW, &t_old);

            // std::cout << "StdinOutWrapper::begin(): recv thread finished.\n";
            return nullptr;
        },
        this);

    // std::cout << "StdinOutWrapper::begin() done.\n";
}

void StdinOutWrapper::end() {
    // std::cout << "StdinOutWrapper::end()...\n";

    recv_running_ = false;
    if (p_in_thread_) {
        pthread_join(p_in_thread_, nullptr);
        p_in_thread_ = 0;
    }

    // std::cout << "StdinOutWrapper::end() done.\n";
}

int StdinOutWrapper::available() {
    return in_buffer_.size();
}

int StdinOutWrapper::peek() {
    if (in_buffer_.size()) {
        return static_cast<int>(in_buffer_.front());
    }
    return -1;
}

int StdinOutWrapper::read() {
    int ret { -1 };

    if (in_buffer_.size()) {
        ret = static_cast<int>(in_buffer_.front());
        pthread_mutex_lock(&in_buffer_mutex_);
        in_buffer_.pop_front();
        pthread_mutex_unlock(&in_buffer_mutex_);
    }
    return ret;
}

size_t StdinOutWrapper::write(const uint8_t data) {
    sigset_t set, old;
    sigfillset(&set);
    pthread_sigmask(SIG_SETMASK, &set, &old);
    ::write(1, &data, 1);
    pthread_sigmask(SIG_SETMASK, &old, nullptr);
    return 1;
}

size_t StdinOutWrapper::write(const uint8_t* buffer, size_t length) {
    sigset_t set, old;
    sigfillset(&set);
    pthread_sigmask(SIG_SETMASK, &set, &old);
    ::write(1, buffer, length);
    pthread_sigmask(SIG_SETMASK, &old, nullptr);
    return length;
}

int StdinOutWrapper::availableForWrite() {
    return std::numeric_limits<int>::max();
}

void StdinOutWrapper::flush() {}


StdinOutWrapper Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;
HardwareSerial Serial3;
HardwareSerial Serial4;
HardwareSerial Serial5;
HardwareSerial Serial6;
HardwareSerial Serial7;
HardwareSerial Serial8;

namespace arduino::posix {
SerialIOStreamAdapter Serial { arduino::Serial };
SerialIOStreamAdapter Serial1 { arduino::Serial1 };
SerialIOStreamAdapter Serial2 { arduino::Serial2 };
SerialIOStreamAdapter Serial3 { arduino::Serial3 };
SerialIOStreamAdapter Serial4 { arduino::Serial4 };
SerialIOStreamAdapter Serial5 { arduino::Serial5 };
SerialIOStreamAdapter Serial6 { arduino::Serial6 };
SerialIOStreamAdapter Serial7 { arduino::Serial7 };
SerialIOStreamAdapter Serial8 { arduino::Serial8 };
} // namespace arduino::posix

SPIClass SPI;
SPIClass SPI1;
SPIClass SPI2;

TwoWire Wire;
TwoWire Wire1;
TwoWire Wire2;
TwoWire Wire3;

CrashReportClass CrashReport;

void (*_VectorsRam[255 + 16])(void);

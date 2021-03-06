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

#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <unistd.h>
#include <termios.h>
#include <poll.h>


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

StdinOutWrapper::StdinOutWrapper() : recv_running_ {} {}

StdinOutWrapper::~StdinOutWrapper() {
    if (recv_running_) {
        recv_running_ = false;
        p_in_thread_->join();
    }
}

void StdinOutWrapper::begin(uint32_t) {
    recv_running_ = true;
    p_in_thread_ = std::make_unique<std::thread>([this]() {
        while (recv_running_) {
            if (key_pressed(100)) {
                std::string line;
                std::getline(std::cin, line); // FIXME: use ncurses?
                line += '\n';
                std::lock_guard<std::mutex> lck(in_mutex_);
                std::copy(line.begin(), line.end(), std::inserter(in_buffer_, in_buffer_.end()));
            }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
        std::cout << "StdinOutWrapper::StdinOutWrapper(): recv thread finished.\n";
    });
}

int StdinOutWrapper::available() {
    std::lock_guard<std::mutex> lck(in_mutex_);
    return in_buffer_.size();
}

int StdinOutWrapper::peek() {
    std::lock_guard<std::mutex> lck(in_mutex_);
    if (in_buffer_.size()) {
        return in_buffer_.front();
    }
    return -1;
}

int StdinOutWrapper::read() {
    int ret { -1 };
    std::lock_guard<std::mutex> lck(in_mutex_);
    if (in_buffer_.size()) {
        ret = static_cast<int>(in_buffer_.front());
        in_buffer_.pop_front();
    }
    return ret;
}

size_t StdinOutWrapper::write(const uint8_t data) {
    std::cout << static_cast<char>(data);
    return 1;
}

size_t StdinOutWrapper::write(const uint8_t* buffer, size_t length) {
    std::cout.write(reinterpret_cast<const char*>(buffer), length);
    return length;
}

int StdinOutWrapper::availableForWrite() {
    return std::numeric_limits<int>::max();
}

void StdinOutWrapper::flush() {
    std::cout.flush();
}

bool StdinOutWrapper::key_pressed(uint32_t timeout_ms) {
    struct pollfd pls[1];
    pls[0].fd = STDIN_FILENO;
    pls[0].events = POLLIN | POLLPRI;
    return ::poll(pls, 1, timeout_ms) > 0;
}


StdinOutWrapper Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;
HardwareSerial Serial3;
HardwareSerial Serial4;
HardwareSerial Serial5;
HardwareSerial Serial6;
HardwareSerial Serial7;
HardwareSerial Serial8;

SPIClass SPI;
SPIClass SPI1;
SPIClass SPI2;

TwoWire Wire;
TwoWire Wire1;
TwoWire Wire2;
TwoWire Wire3;

void (*_VectorsRam[255 + 16])(void);

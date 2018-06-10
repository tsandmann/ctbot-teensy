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

#include "arduino_fixed.h"
#include "Wire.h"

#include <chrono>
#include <iostream>
#include <vector>
#include <string>


namespace arduino {
static auto g_start_time { std::chrono::high_resolution_clock::now() };
static std::vector<bool> g_digital_pins(128);
static std::vector<int16_t> g_analog_pins(128);

uint32_t micros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - g_start_time).count();
}

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

void analogWrite(uint8_t pin, int val) {
    try {
        g_analog_pins.at(pin) = val;
    } catch (const std::exception& e) {
        std::cerr << "arduino::analogWrite(" << static_cast<uint16_t>(pin) << ", " << val << "): " << e.what() << "\n";
    }
}

void analogWriteFrequency(uint8_t, float) {}

uint32_t analogWriteResolution(uint32_t) {
    return 0;
}

void attachInterrupt(uint8_t, void (*)(void), int) {}

void detachInterrupt(uint8_t) {}

uint8_t digitalReadFast(uint8_t pin) {
     try {
        return g_digital_pins.at(pin);
    } catch (const std::exception& e) {
        std::cerr << "arduino::digitalReadFast(" << static_cast<uint16_t>(pin) << "): " << e.what() << "\n";
        return 255;
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

Stream::Stream() : recv_running_ { false } {};

void Stream::setRX(uint8_t) {}

void Stream::setTX(uint8_t) {}

void Stream::begin(uint32_t) {}

int Stream::available() {
    std::lock_guard<std::mutex> lock { in_mutex_ };
    return in_buffer_.size();
}

int Stream::peek() {
    std::lock_guard<std::mutex> lock { in_mutex_ };
    if (in_buffer_.size()) {
        return in_buffer_.front();
    }
    return -1;
}

int Stream::read() {
    int ret { -1 };
    std::lock_guard<std::mutex> lock { in_mutex_ };
    if (in_buffer_.size()) {
        ret = static_cast<int>(in_buffer_.front());
        in_buffer_.pop_front();
    }
    return ret;
}

uint32_t Stream::readBytes(void* buffer, size_t length) {
    char* ptr { reinterpret_cast<char*>(buffer) };

    std::lock_guard<std::mutex> lock { in_mutex_ };
    length = std::min(length, in_buffer_.size());
    for (size_t i { 0 }; i < length; ++i) {
        ptr[i] = in_buffer_.front();
        in_buffer_.pop_front();
    }
    return length;
}

uint32_t Stream::write(const char data) {
    std::cout << data;
    return 1;
}

uint32_t Stream::write(const void* buffer, size_t length) {
    std::cout.write(reinterpret_cast<const char*>(buffer), length);
    return length;
}

void Stream::flush() {
    std::cout.flush();
}

StdinWrapper::StdinWrapper() {
    recv_running_ = true;
    p_recv_thread_ = new std::thread { [this] () {
        while (recv_running_) {
            std::string line;
            std::getline(std::cin, line);
            line += '\n';
            {
                std::lock_guard<std::mutex> lock { in_mutex_ };
                std::copy(line.begin(), line.end(), std::inserter(in_buffer_, in_buffer_.end()));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } };
}

StdinWrapper::~StdinWrapper() {
    recv_running_ = false;
    std::cin.putback('\n');
    if (p_recv_thread_->joinable()) {
        p_recv_thread_->join();
    }
    std::cout << "StdinWrapper::~StdinWrapper(): receiver thread finished.\n";
}

StdinWrapper Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;
HardwareSerial Serial3;
HardwareSerial Serial4;
HardwareSerial Serial5;
HardwareSerial Serial6;

} // namespace arduino

TwoWire Wire2;

extern "C" void setup();

int main(int argc, char** argv) {
    std::cout << "c't-Bot Teensy framework starting...\n";
    setup();
    std::cout << "exit.\n";
    return 0;
}

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
 * @file    Stream.h
 * @brief   Wrapper aroung Arduino Stream library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include "Print.h"
#include "WString.h"


class Stream : public Print {
public:
    constexpr Stream() {}
    void setRX(uint8_t pin);
    void setTX(uint8_t pin);
    void begin(uint32_t baud);

    virtual int available() = 0;
    virtual int peek() = 0;
    virtual int read() = 0;

    bool find(const char* target);
    bool find(const uint8_t* target) {
        return find((const char*) target);
    }
    bool find(const String& target) {
        return find(target.c_str());
    }
    bool find(const char* target, size_t length);
    bool find(const uint8_t* target, size_t length) {
        return find((const char*) target, length);
    }
    bool find(const String& target, size_t length) {
        return find(target.c_str(), length);
    }
    bool findUntil(const char* target, const char* terminator);
    bool findUntil(const uint8_t* target, const char* terminator) {
        return findUntil((const char*) target, terminator);
    }
    bool findUntil(const String& target, const char* terminator) {
        return findUntil(target.c_str(), terminator);
    }
    bool findUntil(const char* target, const String& terminator) {
        return findUntil(target, terminator.c_str());
    }
    bool findUntil(const String& target, const String& terminator) {
        return findUntil(target.c_str(), terminator.c_str());
    }
    bool findUntil(const char* target, size_t targetLen, const char* terminate, size_t termLen);
    bool findUntil(const uint8_t* target, size_t targetLen, const char* terminate, size_t termLen) {
        return findUntil((const char*) target, targetLen, terminate, termLen);
    }
    bool findUntil(const String& target, size_t targetLen, const char* terminate, size_t termLen);
    bool findUntil(const char* target, size_t targetLen, const String& terminate, size_t termLen);
    bool findUntil(const String& target, size_t targetLen, const String& terminate, size_t termLen);
    long parseInt();
    long parseInt(char skipChar);
    float parseFloat();
    float parseFloat(char skipChar);


    size_t readBytes(char* buffer, size_t length);
    size_t readBytes(uint8_t* buffer, size_t length) {
        return readBytes(reinterpret_cast<char*>(buffer), length);
    }

    size_t readBytesUntil(char terminator, char* buffer, size_t length);
    size_t readBytesUntil(char terminator, uint8_t* buffer, size_t length) {
        return readBytesUntil(terminator, (char*) buffer, length);
    }
    String readString(size_t max = 120);
    String readStringUntil(char terminator, size_t max = 120);
    int getReadError() {
        return 0;
    }
    void clearReadError() {}
};

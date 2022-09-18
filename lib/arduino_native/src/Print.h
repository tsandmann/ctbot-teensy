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
 * @file    Print.h
 * @brief   Wrapper aroung Arduino Print library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include "WString.h"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>


class Print;

class Printable {
public:
    virtual size_t printTo(Print& p) const = 0;
};

class Print {
public:
    constexpr Print() = default;
    virtual size_t write(const uint8_t) = 0;
    virtual size_t write(const uint8_t*, size_t);
    virtual int availableForWrite() {
        return 0;
    }
    virtual void flush() {}
    size_t write(const char c) {
        return write(static_cast<uint8_t>(c));
    }
    size_t write(const char* str) {
        return write(str, std::strlen(str));
    }
    size_t write(const char* buffer, size_t size) {
        return write(reinterpret_cast<const uint8_t*>(buffer), size);
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

    size_t print(uint8_t b) {
        return printNumber(b, 10, 0);
    }
    size_t print(int n) {
        return print((long) n);
    }
    size_t print(unsigned int n) {
        return printNumber(n, 10, 0);
    }
    size_t print(long n);
    size_t print(unsigned long n) {
        return printNumber(n, 10, 0);
    }

    size_t print(unsigned char n, int base) {
        return printNumber(n, base, 0);
    }
    size_t print(int n, int base) {
        return (base == 10) ? print(n) : printNumber(n, base, 0);
    }
    size_t print(unsigned int n, int base) {
        return printNumber(n, base, 0);
    }
    size_t print(long n, int base) {
        return (base == 10) ? print(n) : printNumber(n, base, 0);
    }
    size_t print(unsigned long n, int base) {
        return printNumber(n, base, 0);
    }

    size_t print(double n, int digits = 2) {
        return printFloat(n, digits);
    }
    size_t print(const Printable& obj) {
        return obj.printTo(*this);
    }
    size_t println();
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

    size_t println(unsigned char n, int base) {
        return print(n, base) + println();
    }
    size_t println(int n, int base) {
        return print(n, base) + println();
    }
    size_t println(unsigned int n, int base) {
        return print(n, base) + println();
    }
    size_t println(long n, int base) {
        return print(n, base) + println();
    }
    size_t println(unsigned long n, int base) {
        return print(n, base) + println();
    }

    size_t println(double n, int digits = 2) {
        return print(n, digits) + println();
    }
    size_t println(const Printable& obj) {
        return obj.printTo(*this) + println();
    }
    int getWriteError() {
        return 0;
    }
    void clearWriteError() {}
    int printf(const char* format, ...);
    int printf(const __FlashStringHelper* format, ...);

protected:
    size_t printFloat(double n, uint8_t digits);
    size_t printNumber(unsigned long n, uint8_t base, uint8_t sign);
};

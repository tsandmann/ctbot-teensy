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
 * @file    Print.cpp
 * @brief   Wrapper aroung Arduino Print library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#include "Print.h"

size_t Print::write(const uint8_t* buffer, size_t size) {
    if (!buffer) {
        return 0;
    }
    size_t count {};
    while (size--) {
        count += write(*buffer++);
    }
    return count;
}

size_t Print::print(long n) {
    uint8_t sign = 0;

    if (n < 0) {
        sign = '-';
        n = -n;
    }
    return printNumber(n, 10, sign);
}

size_t Print::println() {
    uint8_t buf[2] = { '\r', '\n' };
    return write(buf, 2);
}

size_t Print::printNumber(unsigned long n, uint8_t base, uint8_t sign) {
    uint8_t buf[34];
    uint8_t digit, i;

    if (base == 0) {
        return write(static_cast<uint8_t>(n));
    } else if (base == 1) {
        base = 10;
    }


    if (n == 0) {
        buf[sizeof(buf) - 1] = '0';
        i = sizeof(buf) - 1;
    } else {
        i = sizeof(buf) - 1;
        while (1) {
            digit = n % base;
            buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
            n /= base;
            if (n == 0) {
                break;
            }
            i--;
        }
    }
    if (sign) {
        i--;
        buf[i] = '-';
    }
    return write(buf + i, sizeof(buf) - i);
}

size_t Print::printFloat(double number, uint8_t digits) {
    uint8_t sign = 0;
    size_t count = 0;

    if (std::isnan(number)) {
        return print("nan");
    }
    if (std::isinf(number)) {
        return print("inf");
    }
    if (number > 4294967040.f) {
        return print("ovf"); // constant determined empirically
    }
    if (number < -4294967040.f) {
        return print("ovf"); // constant determined empirically
    }

    // Handle negative numbers
    if (number < 0.0) {
        sign = 1;
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i) {
        rounding *= 0.1;
    }
    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = static_cast<unsigned long>(number);
    double remainder = number - static_cast<double>(int_part);
    count += printNumber(int_part, 10, sign);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        uint8_t n, buf[16], count = 1;
        buf[0] = '.';

        // Extract digits from the remainder one at a time
        if (digits > sizeof(buf) - 1)
            digits = sizeof(buf) - 1;

        while (digits-- > 0) {
            remainder *= 10.0;
            n = static_cast<uint8_t>(remainder);
            buf[count++] = '0' + n;
            remainder -= n;
        }
        count += write(buf, count);
    }
    return count;
}

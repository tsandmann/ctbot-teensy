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
 * @file    WString.h
 * @brief   Wrapper aroung Arduino String library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include <cstring>
#include <cstdint>


class __FlashStringHelper;
class Print;

// An inherited class for holding the result of a concatenation. These
// result objects are assumed to be writable by subsequent concatenations.
class StringSumHelper;

class String {
public:
    String(const char* cstr = nullptr);
    String(const __FlashStringHelper* pgmstr);
    String(const String& str);
    String(String&& rval);
    String(StringSumHelper&& rval);
    String(char c);
    String(unsigned char c);
    String(int, unsigned char base = 10);
    String(unsigned int, unsigned char base = 10);
    String(long, unsigned char base = 10);
    String(unsigned long, unsigned char base = 10);
    String(float num, unsigned char digits = 2);
    String(double num, unsigned char digits = 2) : String(static_cast<float>(num), digits) {}
    ~String();

    unsigned char reserve(unsigned int size);
    unsigned int length() const {
        return len;
    }

    String& copy(const char* cstr, unsigned int length);
    String& copy(const __FlashStringHelper* s) {
        return copy((const char*) s, std::strlen((const char*) s));
    }
    void move(String& rhs);
    String& operator=(const String& rhs);
    String& operator=(const char* cstr);
    String& operator=(const __FlashStringHelper* pgmstr);
    String& operator=(String&& rval);
    String& operator=(StringSumHelper&& rval);
    String& operator=(char c);

    String& append(const String& str);
    String& append(const char* cstr);
    String& append(const __FlashStringHelper* s) {
        return append((const char*) s, strlen((const char*) s));
    }
    String& append(char c);
    String& append(unsigned char c) {
        return append((int) c);
    }
    String& append(int num);
    String& append(unsigned int num);
    String& append(long num);
    String& append(unsigned long num);
    String& append(float num);
    String& append(double num) {
        return append((float) num);
    }
    String& operator+=(const String& rhs) {
        return append(rhs);
    }
    String& operator+=(const char* cstr) {
        return append(cstr);
    }
    String& operator+=(const __FlashStringHelper* pgmstr) {
        return append(pgmstr);
    }
    String& operator+=(char c) {
        return append(c);
    }
    String& operator+=(unsigned char c) {
        return append((int) c);
    }
    String& operator+=(int num) {
        return append(num);
    }
    String& operator+=(unsigned int num) {
        return append(num);
    }
    String& operator+=(long num) {
        return append(num);
    }
    String& operator+=(unsigned long num) {
        return append(num);
    }
    String& operator+=(float num) {
        return append(num);
    }
    String& operator+=(double num) {
        return append(num);
    }

    friend StringSumHelper& operator+(const StringSumHelper& lhs, const String& rhs);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, const char* cstr);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, const __FlashStringHelper* pgmstr);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, char c);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, unsigned char c);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, int num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, unsigned int num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, long num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, unsigned long num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, float num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, double num);
    String& concat(const String& str) {
        return append(str);
    }
    String& concat(const char* cstr) {
        return append(cstr);
    }
    String& concat(const __FlashStringHelper* pgmstr) {
        return append(pgmstr);
    }
    String& concat(char c) {
        return append(c);
    }
    String& concat(unsigned char c) {
        return append((int) c);
    }
    String& concat(int num) {
        return append(num);
    }
    String& concat(unsigned int num) {
        return append(num);
    }
    String& concat(long num) {
        return append(num);
    }
    String& concat(unsigned long num) {
        return append(num);
    }
    String& concat(float num) {
        return append(num);
    }
    String& concat(double num) {
        return append(num);
    }

    int compareTo(const String& s) const;
    unsigned char equals(const String& s) const;
    unsigned char equals(const char* cstr) const;
    unsigned char operator==(const String& rhs) const {
        return equals(rhs);
    }
    unsigned char operator==(const char* cstr) const {
        return equals(cstr);
    }
    unsigned char operator==(const __FlashStringHelper* s) const {
        return equals((const char*) s);
    }
    unsigned char operator!=(const String& rhs) const {
        return !equals(rhs);
    }
    unsigned char operator!=(const char* cstr) const {
        return !equals(cstr);
    }
    unsigned char operator!=(const __FlashStringHelper* s) const {
        return !equals(s);
    }
    unsigned char operator<(const String& rhs) const;
    unsigned char operator>(const String& rhs) const;
    unsigned char operator<=(const String& rhs) const;
    unsigned char operator>=(const String& rhs) const;
    unsigned char equalsIgnoreCase(const String& s) const;
    unsigned char startsWith(const String& prefix) const;
    unsigned char startsWith(const String& prefix, unsigned int offset) const;
    unsigned char endsWith(const String& suffix) const;

    char charAt(unsigned int index) const;
    void setCharAt(unsigned int index, char c);
    char operator[](unsigned int index) const;
    char& operator[](unsigned int index);
    void getBytes(unsigned char* buf, unsigned int bufsize, unsigned int index = 0) const;
    void toCharArray(char* buf, unsigned int bufsize, unsigned int index = 0) const {
        getBytes((unsigned char*) buf, bufsize, index);
    }
    const char* c_str() const {
        return buffer;
    }

    int indexOf(char ch) const;
    int indexOf(char ch, unsigned int fromIndex) const;
    int indexOf(const String& str) const;
    int indexOf(const String& str, unsigned int fromIndex) const;
    int lastIndexOf(char ch) const;
    int lastIndexOf(char ch, unsigned int fromIndex) const;
    int lastIndexOf(const String& str) const;
    int lastIndexOf(const String& str, unsigned int fromIndex) const;
    String substring(unsigned int beginIndex) const;
    String substring(unsigned int beginIndex, unsigned int endIndex) const;

    String& replace(char find, char replace);
    String& replace(const String& find, const String& replace);
    String& remove(unsigned int index);
    String& remove(unsigned int index, unsigned int count);
    String& toLowerCase(void);
    String& toUpperCase(void);
    String& trim(void);

    long toInt(void) const;
    float toFloat(void) const;

protected:
    char* buffer; // the actual char array
    unsigned int capacity; // the array length minus one (for the '\0')
    unsigned int len; // the String length (not counting the '\0')
    unsigned char flags; // unused, for future features

    void init();
    unsigned char changeBuffer(unsigned int maxStrLen);
    String& append(const char* cstr, unsigned int length);

private:
    // allow for "if (s)" without the complications of an operator bool().
    // for more information http://www.artima.com/cppsource/safebool.html
    typedef void (String::*StringIfHelperType)() const;
    void StringIfHelper() const {}

    // FIXME: check this
    template <typename T>
    static void toString(T value, char* dest, const unsigned int radix) {
        const char digit[] = "0123456789ABCDEFGHIJKLMNOPRSTUVWXYZ";
        char stack[32];

        int64_t quot, rem;
        size_t digits = 0;

        do {
            quot = value / radix;
            rem = value % radix;

            stack[digits] = digit[rem];
            value = quot;
            digits++;
        } while (value);

        int i = 0;
        while (digits--) {
            dest[i++] = stack[digits];
        }

        dest[i] = 0;
    }

public:
    operator StringIfHelperType() const {
        return buffer ? &String::StringIfHelper : 0;
    }
};

class StringSumHelper : public String {
public:
    StringSumHelper(const String& s) : String(s) {}
    StringSumHelper(const char* p) : String(p) {}
    StringSumHelper(const __FlashStringHelper* pgmstr) : String(pgmstr) {}
    StringSumHelper(char c) : String(c) {}
    StringSumHelper(unsigned char c) : String(c) {}
    StringSumHelper(int num) : String(num, 10) {}
    StringSumHelper(unsigned int num) : String(num, 10) {}
    StringSumHelper(long num) : String(num, 10) {}
    StringSumHelper(unsigned long num) : String(num, 10) {}
};

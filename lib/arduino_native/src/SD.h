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
 * @file    SD.h
 * @brief   Wrapper aroung Arduino SD library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include "arduino_fixed.h"

#include <fstream>


static constexpr uint8_t O_READ { 0X01 };
static constexpr uint8_t O_RDONLY { O_READ };
static constexpr uint8_t O_WRITE { 0X02 };
static constexpr uint8_t O_WRONLY { O_WRITE };
static constexpr uint8_t O_RDWR { O_READ | O_WRITE };
static constexpr uint8_t O_ACCMODE { O_READ | O_WRITE };
static constexpr uint8_t O_APPEND { 0X04 };
static constexpr uint8_t O_SYNC { 0X08 };
static constexpr uint8_t O_CREAT { 0X10 };
static constexpr uint8_t O_EXCL { 0X20 };
static constexpr uint8_t O_TRUNC { 0X40 };


class File : public Stream {
    char name_[13];
    std::fstream file_;

public:
    File() = default;
    File(std::fstream file, const char* name);
    ~File();
    virtual size_t write(const uint8_t) override;
    virtual size_t write(const uint8_t* buf, size_t size) override;
    virtual int read() override;
    virtual int peek() override;
    virtual int available() override;
    virtual void flush() override;
    int read(void* buf, uint16_t nbyte);
    bool seek(uint32_t pos);
    // uint32_t position();
    uint32_t size();
    void close();
    operator bool();
    const char* name() {
        return name_;
    }

    bool isDirectory() {
        return false;
    }
    File openNextFile(uint8_t mode = 1) {
        (void) mode;
        return File {};
    }
    // void rewindDirectory();

    using Print::write;
};


class SDClass {
public:
    bool begin(uint8_t = 0) {
        return true;
    }

    File open(const char* name, uint8_t mode = O_READ);

    bool exists(const char* name);

    bool mkdir(const char* name);

    bool remove(const char* name);

    bool rmdir(const char* name);
};

extern SDClass SD;

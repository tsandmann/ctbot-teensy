/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    serial_io.h
 * @brief   Teensy serialport abstraction
 * @author  Timo Sandmann
 * @date    16.10.2021
 */

#pragma once

#include "arduino_freertos.h"

#include <cstddef>
#include <cstdint>
#include <string_view>


namespace arduino {

class SerialIO {
public:
    SerialIO() = default;
    virtual ~SerialIO() = default;

    bool begin(uint32_t baud) {
        return begin(baud, 0);
    }

    bool begin(uint32_t baud, uint16_t format) {
        return begin(baud, format, 0, 0);
    }

    virtual bool begin(uint32_t baud, uint16_t format, size_t rx_buf_size, size_t tx_buf_size) = 0;

    virtual void end() = 0;

    virtual bool setRX(uint8_t pin) = 0;

    virtual bool setTX(uint8_t pin, bool opendrain = false) = 0;

    virtual size_t available() const = 0;

    virtual int peek() const = 0;

    int read() const {
        return read(false);
    }

    virtual int read(bool blocking) const = 0;

    size_t read(void* p_data, size_t length) const {
        return read(p_data, length, false);
    }

    virtual size_t read(void* p_data, const size_t length, bool blocking) const = 0;

    virtual size_t availableForWrite() const = 0;

    size_t write(uint8_t c) {
        return write(c, true);
    }

    size_t write(uint8_t c, bool blocking) {
        return write(&c, 1, blocking);
    }

    size_t write(const void* p_data, size_t length) {
        return write(p_data, length, true);
    }

    size_t write(const std::string_view& str) {
        return write(str.data(), str.length(), true);
    }

    size_t write(const std::string_view& str, bool blocking) {
        return write(str.data(), str.length(), blocking);
    }

    virtual size_t write(const void* p_data, size_t length, bool blocking) = 0;

    virtual void write_direct(uint8_t c) const = 0;

    void write_direct(const std::string_view& str) const {
        for (const auto& c : str) {
            write_direct(c);
        }
    }

    virtual void flush() const = 0;

    virtual void flush_direct() const = 0;

    virtual void clear() = 0;

    virtual Stream& get_stream() = 0;
};


class SerialIOStreamAdapter : public SerialIO {
protected:
    Stream& stream_;

public:
    SerialIOStreamAdapter(Stream& stream) : stream_ { stream } {}

    virtual bool begin(uint32_t, uint16_t, size_t, size_t) override {
        return true;
    }

    virtual void end() override {}

    virtual bool setRX(uint8_t) override {
        return true;
    }

    virtual bool setTX(uint8_t, bool = false) {
        return true;
    }

    virtual size_t available() const override {
        return stream_.available();
    }

    int peek() const override {
        return stream_.peek();
    }

    virtual int read(bool) const override {
        return stream_.read();
    }

    virtual size_t read(void* p_data, size_t length, bool) const override {
        return stream_.readBytes(reinterpret_cast<char*>(p_data), length);
    }

    virtual size_t availableForWrite() const override {
        return stream_.availableForWrite();
    }

    virtual size_t write(const void* p_data, size_t length, bool) override {
        return stream_.write(reinterpret_cast<const uint8_t*>(p_data), length);
    }

    virtual void write_direct(uint8_t c) const override {
        stream_.write(c);
        flush_direct();
    }

    virtual void flush() const override {
        stream_.flush();
    }

    virtual void flush_direct() const override {
        stream_.flush();
    }

    virtual void clear() override {}

    virtual Stream& get_stream() override {
        return stream_;
    }
};
} // namespace arduino

/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    serial_connection_teensy.cpp
 * @brief   Abstraction layer for serial communication on Teensy devices
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "serial_connection_teensy.h"
#include "timer.h"

#include <arduino_fixed.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <cstring>
#include <cstddef>
#include <streambuf>
#include <sstream>


namespace ctbot {

struct streambuf_helper {
    template <class T>
    static const char* get_data(T& buf) {
        struct helper : public T {
            static const char* _get_data(T& buf) {
                auto& buffer(reinterpret_cast<helper&>(buf));
                return buffer.gptr(); // FIXME: double check
            }
        };

        return helper::_get_data(buf);
    }
};

decltype(SerialConnectionTeensy::wait_callback_) SerialConnectionTeensy::wait_callback_(nullptr);

// FIXME: generalize for other serial ports
SerialConnectionTeensy::SerialConnectionTeensy(const uint8_t serial_port) : mutex_ { xSemaphoreCreateMutex() }, io_stream_ { serial_port == 0 ? arduino::Serial : arduino::Serial } {
    io_stream_.begin(CtBotConfig::UART0_BAUDRATE);
}

uint16_t SerialConnectionTeensy::wait_for_data(const uint16_t size, const uint16_t timeout_ms) noexcept {
    uint16_t bytes_available { static_cast<uint16_t>(available()) };
    if (bytes_available >= size) {
        return size;
    }

    const auto start(Timer::get_ms());
    auto now(start);
    auto running_ms(now - start);

    while ((bytes_available < size) && ((! timeout_ms) || (running_ms < timeout_ms))) {
        if (wait_callback_) {
            wait_callback_(this);
        }
        vTaskDelay(1);
        bytes_available = available();
        now = Timer::get_ms();
        running_ms = now - start;
    }

    return std::min<uint16_t>(bytes_available, size);
}

size_t SerialConnectionTeensy::available() const {
    return io_stream_.available();
}

size_t SerialConnectionTeensy::receive(void* data, const size_t size) {
    if (! size) {
        return 0;
    }

    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
        const size_t ret { io_stream_.readBytes(static_cast<char*>(data), size) };
        xSemaphoreGive(mutex_);
        return ret;
    }
    return 0;
}

size_t SerialConnectionTeensy::receive(std::streambuf& buf, const size_t size) {
    if (! size) {
        return 0;
    }

    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
        uint16_t i;
        for (i = 0U; i < size; ++i) {
            buf.sputc(io_stream_.read());
        }
        xSemaphoreGive(mutex_);
        return i;
    }
    return 0;
}

size_t SerialConnectionTeensy::receive_until(void* data, const char delim, const size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n { 0 };
    char* ptr(reinterpret_cast<char*>(data));
    do {
        int c { -1 };
        if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
            c = io_stream_.read();
            xSemaphoreGive(mutex_);
        } else {
            return n;
        }
        if (c >= 0) {
            *ptr = c;
            ++n;
        } else {
            if (wait_callback_) {
                wait_callback_(this);
            }
            vTaskDelay(1);
        }
    } while (*ptr++ != delim && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_until(void* data, const std::string& delim, const size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n { 0 };
    char* ptr(reinterpret_cast<char*>(data));
    do {
        int c { -1 };
        if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
            c = io_stream_.read();
            xSemaphoreGive(mutex_);
        } else {
            return n;
        }
        if (c >= 0) {
            *ptr = c;
            ++ptr;
            ++n;
        } else {
            if (wait_callback_) {
                wait_callback_(this);
            }
            vTaskDelay(1);
        }
    } while (std::strncmp(reinterpret_cast<const char*>(data), delim.c_str(), n) && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_until(std::streambuf& buf, const char delim, const size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n { 0 };
    char tmp;
    do {
        int c { -1 };
        if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
            c = io_stream_.read();
            xSemaphoreGive(mutex_);
        } else {
            return n;
        }
        if (c >= 0) {
            tmp = c;
            buf.sputc(tmp);
            ++n;
        } else {
            tmp = 0;
            if (wait_callback_) {
                wait_callback_(this);
            }
            vTaskDelay(1);
        }
    } while (tmp != delim && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_until(std::streambuf& buf, const std::string& delim, const size_t maxsize) {
    auto& buffer(reinterpret_cast<std::stringbuf&>(buf));
    const auto size16(static_cast<uint16_t>(maxsize));

    uint16_t n { 0 };
    do {
        int c { -1 };
        if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
            c = io_stream_.read();
            xSemaphoreGive(mutex_);
        } else {
            return n;
        }
        if (c >= 0) {
            buf.sputc(c);
            ++n;
        } else {
            if (wait_callback_) {
                wait_callback_(this);
            }
            vTaskDelay(1);
        }
    } while (std::strncmp(streambuf_helper::get_data(buffer), delim.c_str(), n) && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_async(void* data, const size_t size, const uint32_t timeout_ms) {
    auto ptr(reinterpret_cast<uint8_t*>(data));
    const auto timeout(static_cast<uint16_t>(timeout_ms));

    const auto avail(available());
    const auto to_read(std::min(avail, size));
    size_t done { 0 };
    if (to_read) {
        done = receive(ptr, to_read);
        ptr += done;
    }

    const auto n(wait_for_data(size - done, timeout));
    done += receive(ptr, n);

    return done;
}

size_t SerialConnectionTeensy::receive_async(std::streambuf& buf, const size_t size, const uint32_t timeout_ms) {
    const auto timeout(static_cast<uint16_t>(timeout_ms));

    const auto avail(available());
    const auto to_read(std::min(avail, size));
    size_t done { 0 };
    if (to_read) {
        done = receive(buf, to_read);
    }

    const auto n(wait_for_data(size - done, timeout));
    done += receive(buf, n);

    return done;
}

size_t SerialConnectionTeensy::send(const void* data, const size_t size) {
    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
        const size_t ret { io_stream_.write(reinterpret_cast<const uint8_t*>(data), size) };
        xSemaphoreGive(mutex_);
        return ret;
    }
    return 0;
}

size_t SerialConnectionTeensy::send(std::streambuf& buf, const size_t size) {
    const auto size16(static_cast<uint16_t>(size));
    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
        for (auto i(0U); i < size16; ++i) {
            io_stream_.write(static_cast<uint8_t>(buf.sbumpc()));
        }
        xSemaphoreGive(mutex_);
        return size;
    }
    return 0;
}

int SerialConnectionTeensy::peek() const {
    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
        const int ret { io_stream_.peek() };
        xSemaphoreGive(mutex_);
        return ret;
    }
    return -1;
}

void SerialConnectionTeensy::flush() {
    if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY)) {
        io_stream_.flush();
        xSemaphoreGive(mutex_);
    }
}

} /* namespace ctbot */

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

#include <cstring>
#include <cstddef>
#include <streambuf>
#include <sstream>
#include <thread>


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

/**
 * @brief Get the serial port object
 * @param[in] serial_port: Number of serial port
 * @return Reference to serial port driver
 */
static constexpr arduino::Stream& get_serial_port(const uint8_t serial_port) {
    if (serial_port == 1) {
        return arduino::Serial1;
    } else if (serial_port == 2) {
        return arduino::Serial2;
    } else if (serial_port == 3) {
        return arduino::Serial3;
    } else if (serial_port == 4) {
        return arduino::Serial4;
    } else if (serial_port == 5) {
        return arduino::Serial5;
    } else if (serial_port == 6) {
        return arduino::Serial6;
    } else {
        return arduino::Serial;
    }
}

decltype(SerialConnectionTeensy::wait_callback_) SerialConnectionTeensy::wait_callback_(nullptr);

SerialConnectionTeensy::SerialConnectionTeensy(const uint8_t serial_port, const uint8_t pin_rx, const uint8_t pin_tx, const uint32_t baud_rate)
    : io_stream_ { get_serial_port(serial_port) } {
    if (serial_port > 0) {
        arduino::HardwareSerial& hw_serial { reinterpret_cast<arduino::HardwareSerial&>(io_stream_) };
        if (pin_rx < 255U) {
            hw_serial.setRX(pin_rx);
        }
        if (pin_tx < 255U) {
            hw_serial.setTX(pin_tx);
        }
        hw_serial.begin(baud_rate);
    } else {
        /* for the USB serial port there is no need to call begin() or initialize anything */
    }
}

SerialConnectionTeensy::~SerialConnectionTeensy() {}

uint16_t SerialConnectionTeensy::wait_for_data(const uint16_t size, const uint16_t timeout_ms) {
    uint16_t bytes_available { static_cast<uint16_t>(available()) };
    if (bytes_available >= size) {
        return size;
    }

    const auto start { std::chrono::system_clock::now() };
    auto now { start };
    auto running { now - start };

    while ((bytes_available < size) && ((!timeout_ms) || (std::chrono::duration_cast<std::chrono::milliseconds>(running).count() < timeout_ms))) {
        if (wait_callback_) {
            wait_callback_(this);
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1us);
        bytes_available = available();
        now = std::chrono::system_clock::now();
        running = now - start;
    }

    return std::min<uint16_t>(bytes_available, size);
}

size_t SerialConnectionTeensy::available() const {
    return io_stream_.available();
}

size_t SerialConnectionTeensy::receive(void* data, const size_t size) {
    if (!size) {
        return 0;
    }

    std::unique_lock<std::mutex> mlock(mutex_);
    return io_stream_.readBytes(static_cast<char*>(data), size);
}

size_t SerialConnectionTeensy::receive(std::streambuf& buf, const size_t size) {
    if (!size) {
        return 0;
    }

    uint16_t i;
    std::unique_lock<std::mutex> mlock(mutex_);
    for (i = 0U; i < size; ++i) {
        buf.sputc(io_stream_.read());
    }
    return i;
}

size_t SerialConnectionTeensy::receive_until(void* data, const char delim, const size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n {};
    char* ptr { reinterpret_cast<char*>(data) };
    do {
        int c { -1 };
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            c = io_stream_.read();
        }
        if (c >= 0) {
            *ptr = c;
            ++n;
        } else {
            if (wait_callback_) {
                wait_callback_(this);
            }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1us);
        }
    } while (*ptr++ != delim && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_until(void* data, const std::string& delim, const size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n {};
    char* ptr { reinterpret_cast<char*>(data) };
    do {
        int c { -1 };
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            c = io_stream_.read();
        }
        if (c >= 0) {
            *ptr = c;
            ++ptr;
            ++n;
        } else {
            if (wait_callback_) {
                wait_callback_(this);
            }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1us);
        }
    } while (std::strncmp(reinterpret_cast<const char*>(data), delim.c_str(), n) && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_until(std::streambuf& buf, const char delim, const size_t maxsize) {
    const auto size16 { static_cast<uint16_t>(maxsize) };
    uint16_t n {};
    char tmp;
    do {
        int c { -1 };
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            c = io_stream_.read();
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
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1us);
        }
    } while (tmp != delim && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_until(std::streambuf& buf, const std::string& delim, const size_t maxsize) {
    auto& buffer { reinterpret_cast<std::stringbuf&>(buf) };
    const auto size16 { static_cast<uint16_t>(maxsize) };

    uint16_t n {};
    do {
        int c { -1 };
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            c = io_stream_.read();
        }
        if (c >= 0) {
            buf.sputc(c);
            ++n;
        } else {
            if (wait_callback_) {
                wait_callback_(this);
            }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1us);
        }
    } while (std::strncmp(streambuf_helper::get_data(buffer), delim.c_str(), n) && n < size16);

    return n;
}

size_t SerialConnectionTeensy::receive_async(void* data, const size_t size, const uint32_t timeout_ms) {
    auto ptr { reinterpret_cast<uint8_t*>(data) };
    const auto timeout { static_cast<uint16_t>(timeout_ms) };

    const auto avail { available() };
    const auto to_read { std::min(avail, size) };
    size_t done {};
    if (to_read) {
        done = receive(ptr, to_read);
        ptr += done;
    }

    const auto n { wait_for_data(size - done, timeout) };
    done += receive(ptr, n);

    return done;
}

size_t SerialConnectionTeensy::receive_async(std::streambuf& buf, const size_t size, const uint32_t timeout_ms) {
    const auto timeout { static_cast<uint16_t>(timeout_ms) };

    const auto avail { available() };
    const auto to_read { std::min(avail, size) };
    size_t done {};
    if (to_read) {
        done = receive(buf, to_read);
    }

    const auto n { wait_for_data(size - done, timeout) };
    done += receive(buf, n);

    return done;
}

size_t SerialConnectionTeensy::send(const void* data, const size_t size) {
    std::unique_lock<std::mutex> mlock(mutex_);
    return io_stream_.write(reinterpret_cast<const uint8_t*>(data), size);
}

size_t SerialConnectionTeensy::send(const std::string_view& sv) {
    std::unique_lock<std::mutex> mlock(mutex_);
    return io_stream_.write(sv.data(), sv.size());
}

size_t SerialConnectionTeensy::send(std::streambuf& buf, const size_t size) {
    const auto size16 { static_cast<uint16_t>(size) };
    std::unique_lock<std::mutex> mlock(mutex_);
    for (auto i(0U); i < size16; ++i) {
        io_stream_.write(static_cast<uint8_t>(buf.sbumpc()));
    }
    return size;
}

int SerialConnectionTeensy::peek() {
    std::unique_lock<std::mutex> mlock(mutex_);
    return io_stream_.peek();
}

void SerialConnectionTeensy::flush() {
    std::unique_lock<std::mutex> mlock(mutex_);
    io_stream_.flush();
}

} // namespace ctbot

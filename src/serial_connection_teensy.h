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
 * @file    serial_connection_teensy.h
 * @brief   Abstraction layer for serial communication on Teensy devices
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include <arduino_fixed.h>
#include <streambuf>


namespace ctbot {

/**
 * @brief Abstraction layer for serial communication on Teensy devices
 *
 * @startuml{SerialConnectionTeensy.png}
 *  !include serial_connection_teensy.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class SerialConnectionTeensy {
    // FIXME: maybe this class can be simplified a lot...
protected:
    void* mutex_;
    arduino::Stream& io_stream_;

    static void (*wait_callback_)(const void*);

    uint16_t wait_for_data(const uint16_t size, const uint16_t timeout_ms);

public:
    /**
     * @brief Construct a new SerialConnectionTeensy object
     * @param[in] serial_port: ID of underlying serial port to use; 0 for USB serial port emulation
     * @param[in] baud_rate: Baud rate of serial port to use, if serial_port > 0
     */
    SerialConnectionTeensy(const uint8_t serial_port, const uint32_t baud_rate) : SerialConnectionTeensy(serial_port, 255U, 255U, baud_rate) {}

    /**
     * @brief Construct a new SerialConnectionTeensy object
     * @param[in] serial_port: ID of underlying serial port to use; 0 for USB serial port emulation
     * @param[in] pin_rx: Number of pin to use for RX line
     * @param[in] pin_tx: Number of pin to use for TX line
     * @param[in] baud_rate: Baud rate of serial port to use, if serial_port > 0
     */
    SerialConnectionTeensy(const uint8_t serial_port, const uint8_t pin_rx, const uint8_t pin_tx, const uint32_t baud_rate);

    /**
     * @brief Destroy the SerialConnectionTeensy object
     */
    ~SerialConnectionTeensy();

    /**
     * @brief Copy constructor
     * @note Deleted to prohibit copies of this instance
     */
    SerialConnectionTeensy(const SerialConnectionTeensy&) = delete;

    /**
     * @brief Copy assignment operator
     * @return Reference to this instance
     * @note Deleted to prohibit copies of this instance
     */
    SerialConnectionTeensy& operator=(const SerialConnectionTeensy&) = delete;

    /**
     * @brief Set a callback function which is called in case of waiting for incoming data
     * @param[in] callback: Function pointer to callback function
     */
    static void set_wait_callback(decltype(wait_callback_) callback) {
        wait_callback_ = callback;
    }

    /**
     * @return Number of bytes available in receive buffer
     */
    size_t available() const;

    /**
     * @brief Read size bytes from connection into a plain buffer
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] size: Number of bytes to read
     * @return Number of bytes actually read
     */
    size_t receive(void* data, const size_t size);

    /**
     * @brief Read size bytes from connection into a streambuffer
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] size: Number of bytes to read
     * @return Number of bytes actually read
     */
    size_t receive(std::streambuf& buf, const size_t size);

    /**
     * @brief Read bytes from connection until a delimiter char into a plain buffer
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] delim: Delimiter which is used as a stop character for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    size_t receive_until(void* data, const char delim, const size_t maxsize);

    /**
     * @brief Read bytes from connection until a delimiter string into a plain buffer
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] delim: Reference to a delimiter string which is used as a stop sequence for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    size_t receive_until(void* data, const std::string& delim, const size_t maxsize);

    /**
     * @brief Read bytes from connection until a delimiter char into a streambuffer
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] delim: Delimiter which is used as a stop character for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    size_t receive_until(std::streambuf& buf, const char delim, const size_t maxsize);

    /**
     * @brief Read bytes from connection until a delimiter string into a streambuffer
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] delim: Reference to a delimiter string which is used as a stop sequence for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    size_t receive_until(std::streambuf& buf, const std::string& delim, const size_t maxsize);

    /**
     * @brief Read size bytes from connection into a plain buffer with a timeout
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] size: Number of bytes to read
     * @param[in] timeout_ms: Timeout while waiting for data
     * @return Number of bytes actually read
     */
    size_t receive_async(void* data, const size_t size, const uint32_t timeout_ms);

    /**
     * @brief Read size bytes from connection into a streambuffer with a timeout
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] size: Number of bytes to read at most
     * @param[in] timeout_ms: Timeout while waiting for data
     * @return Number of bytes actually read
     */
    size_t receive_async(std::streambuf& buf, const size_t size, const uint32_t timeout_ms);

    /**
     * @brief Write size bytes to connection from a plain buffer
     * @param[in] data: Pointer to buffer with data to write out
     * @param[in] size: Number of bytes to write
     * @return Number of bytes actually written
     */
    size_t send(const void* data, const size_t size);

    /**
     * @brief Write size bytes to connection from a streambuffer
     * @param[in] buf: Reference to streambuffer to read from
     * @param[in] size: Number of bytes to write
     * @return Number of bytes actually written
     */
    size_t send(std::streambuf& buf, const size_t size);

    /**
     * @brief Get the next byte received without removing it from the receive buffer
     * @return The next byte or -1, if receive buffer is empty
     * @note Just calls peek() of the underlying connection stream
     */
    int peek() const;

    /**
     * @brief Wait for any outstanding transmission to complete
     */
    void flush();
};

} // namespace ctbot

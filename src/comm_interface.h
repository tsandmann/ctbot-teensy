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
 * @file    comm_interface.h
 * @brief   Communication interface classes of c't-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "circular_buffer.h"
#include "memory_pool.hpp"
#include "static_allocator.hpp"
#include "namespace_alias.hpp"

#include "avr/pgmspace.h"

#include <cstdint>
#include <array>
#include <string>
#include <string_view>
#include <memory>
#include <mutex>
#include <type_traits>


namespace ctbot {

class CtBot;
class CmdParser;
class SerialConnectionTeensy;

/**
 * @brief Abstraction layer for communication services
 *
 * @startuml{CommInterface.png}
 *  !include comm_interface.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class CommInterface {
protected:
    friend class CtBot;
    friend class TFTDisplay;

    static constexpr size_t INPUT_BUFFER_SIZE { 64 }; /**< Size of input buffer in byte */
    static constexpr size_t OUTPUT_QUEUE_SIZE { 128 }; /**< Size of output queue in number of elements */
    static constexpr size_t BUFFER_CHUNK_SIZE { 4 };
    static constexpr uint16_t INPUT_TASK_PERIOD_MS { 20 }; /**< Scheduling period of input task in ms */
    static constexpr uint8_t INPUT_TASK_PRIORITY { 5 }; /**< Priority of input task */
    static constexpr uint32_t INPUT_TASK_STACK_SIZE { 1536 }; /**< stack size of input task */
    static constexpr uint16_t OUTPUT_TASK_PERIOD_MS { 20 }; /**< Scheduling period of output task in ms */
    static constexpr uint8_t OUTPUT_TASK_PRIORITY { 4 }; /**< Priority of output task */
    static constexpr uint32_t OUTPUT_TASK_STACK_SIZE { 1024 }; /**< stack size of output task */

    struct OutBufferElement {
        const std::string* p_str_;
        char character_;
    } __attribute__((packed));

    DMAMEM static memory::static_allocator_storage<INPUT_BUFFER_SIZE + OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement) + 16> buffer_storage_;
    using static_pool_t = memory::memory_pool<memory::array_pool, memory::static_allocator>;
    static static_pool_t mem_pool_;

    SerialConnectionTeensy& io_;
    bool echo_;
    int error_;
    char* p_input_;
    CircularBuffer<OutBufferElement, OUTPUT_QUEUE_SIZE> output_queue_;
    uint16_t input_task_;
    uint16_t output_task_;
    std::array<char, INPUT_BUFFER_SIZE>* p_input_buffer_;

    /**
     * @brief Worker task implementation that processes incoming data
     * @note Pure virtual method, override for specialized implementations
     */
    virtual void run_input() = 0;

    void run_output();

    FLASHMEM size_t queue_debug_msg(const char c, const std::string* p_str, const bool block);

    FLASHMEM static size_t get_format_size(const char* format, ...) __attribute__((format(printf, 1, 2)));

    FLASHMEM static std::string* create_formatted_string(const size_t size, const char* format, ...);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    template <typename... Args>
    FLASHMEM static std::string* string_format(const char* format, const Args&... args) {
        const auto size { get_format_size(format, args...) + 1 };
        if (size > 0) {
            return create_formatted_string(size, format, args...);
        } else {
            return nullptr;
        }
    }
#pragma GCC diagnostic pop

public:
    enum class Color : uint8_t {
        BLACK = 0,
        RED = 1,
        GREEN = 2,
        YELLOW = 3,
        BLUE = 4,
        MAGENTA = 5,
        CYAN = 6,
        WHITE = 7,
    };

    enum class Attribute : uint8_t {
        NORMAL = 0,
        BOLD = 1,
        UNDERLINE = 4,
        BLINK = 5,
        REVERSE = 7,
    };

    /**
     * @brief Construct a new CommInterface object
     * @param[in] io_connection: Reference to SerialConnection to use
     * @param[in] enable_echo: character echo mode for console, defaults to false
     */
    FLASHMEM CommInterface(SerialConnectionTeensy& io_connection, bool enable_echo = false);

    /**
     * @brief Destroy the CommInterface object
     */
    FLASHMEM virtual ~CommInterface();

    /**
     * @return Current character echo mode setting
     */
    auto get_echo() const {
        return echo_;
    }

    /**
     * @return Last error code
     */
    auto get_error() const {
        return error_;
    }

    /**
     * @brief Reset the stored error code
     */
    void reset_error() {
        error_ = 0;
    }

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     * @note Pure virtual method, override for specialized implementations
     */
    FLASHMEM virtual void set_echo(bool value) = 0;

    void set_color(const Color fg, const Color bg);

    void set_attribute(const Attribute a);

    /**
     * @brief Write a character out to a SerialConnection
     * @param[in] c: Character to write
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const char c, const bool block) {
        return queue_debug_msg(c, nullptr, block);
    }

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Pointer to message as C-string
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const char* str, const bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] p_str: Message as pointer to std::string, memory will be released after print
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const std::string* p_str, const bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Reference to message as string
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const std::string& str, const bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Message as string
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(std::string&& str, const bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Reference to message as string_view
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const std::string_view& str, const bool block);

    /**
     * @brief Write an integer or float out to SerialConnection
     * @tparam T: Type of integer or float
     * @param[in] v: Value
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value>>
    FLASHMEM size_t debug_print(const T v, const bool block) {
        return debug_print(std::to_string(v), block);
    }

    /**
     * @brief Write an formatted string (like std::printf) out to SerialConnection
     * @tparam BLOCK: Switch blocking mode
     * @tparam Args: Values to print
     * @param[in] format: Format string
     * @return Number of characters written
     */
    template <bool BLOCK = false, typename... Args>
    FLASHMEM size_t debug_printf(const char* format, const Args&... args) {
        return debug_print(string_format(format, args...), BLOCK);
    }

    /**
     * @brief Wait for any outstanding transmission on the serial connection to complete
     */
    FLASHMEM void flush();
};


/**
 * @brief Specialization of CommInterface that uses a CmdParser to build a simple command line interface
 *
 * @startuml{CommInterfaceCmdParser.png}
 *  !include comm_interface.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class CommInterfaceCmdParser : public CommInterface {
protected:
    CmdParser& cmd_parser_;
    size_t history_view_;

    /**
     * @brief Worker task implementation that processes incoming data
     * @details The incoming data is parsed with the associated CmdParser and
     * registered commands are executed accordingly.
     */
    FLASHMEM virtual void run_input() override;

    FLASHMEM void clear_line();
    FLASHMEM void update_line(const std::string_view& line);

public:
    /**
     * @brief Construct a new CommInterfaceCmdParser object
     * @param[in] io_connection: Reference to SerialConnection to use
     * @param[in] parser: Reference to CmdParser to use
     * @param[in] enable_echo: character echo mode for console, defaults to false
     */
    FLASHMEM CommInterfaceCmdParser(SerialConnectionTeensy& io_connection, CmdParser& parser, bool enable_echo = false);

    /**
     * @brief Destroy the CommInterfaceCmdParser object
     */
    virtual ~CommInterfaceCmdParser() override = default;

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     */
    virtual void set_echo(bool value) override;
};

} // namespace ctbot

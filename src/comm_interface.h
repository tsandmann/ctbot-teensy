/*
 * This file is part of the ct-Bot teensy framework.
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
 * @brief   Communication interface classes of ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "logger.h"

#include "circular_buffer.h"
#define FOONATHAN_HAS_EXCEPTION_SUPPORT 0
#include "memory_pool.hpp"
#include "namespace_alias.hpp"
#include "static_allocator.hpp"

#include "avr/pgmspace.h"

#include <array>
#include <concepts>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <type_traits>


namespace arduino {
class SerialIO;
}

namespace ctbot {
class CtBot;
class CmdParser;

static constexpr size_t calc_pool_storage_size(size_t size, size_t align = sizeof(std::max_align_t)) {
    return (size + 16 + align - 1) / align * align;
}

/**
 * @brief Abstraction layer for communication services
 *
 * @startuml{CommInterface.png}
 *  !include comm_interface.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class CommInterface : public LoggerTarget {
    static constexpr bool DEBUG_ { false };

protected:
    friend class CtBot;
    friend class TFTDisplay;

    static constexpr size_t INPUT_BUFFER_SIZE { 64 }; /**< Size of input buffer in byte */
    static constexpr size_t OUTPUT_QUEUE_SIZE { 128 }; /**< Size of output queue in number of elements */
    static constexpr size_t BUFFER_CHUNK_SIZE { 4 };
    static constexpr uint16_t INPUT_TASK_PERIOD_MS { 20 }; /**< Scheduling period of input task in ms */
    static constexpr uint8_t INPUT_TASK_PRIORITY { 5 }; /**< Priority of input task */
    static constexpr uint32_t INPUT_TASK_STACK_SIZE { 4096 }; /**< stack size of input task */
    static constexpr uint16_t OUTPUT_TASK_PERIOD_MS { 20 }; /**< Scheduling period of output task in ms */
    static constexpr uint8_t OUTPUT_TASK_PRIORITY { 4 }; /**< Priority of output task */
    static constexpr uint32_t OUTPUT_TASK_STACK_SIZE { 2048 }; /**< stack size of output task */

    struct OutBufferElement {
        const std::string* p_str_;
        char character_;
    } __attribute__((packed));

    DMAMEM alignas(8) static inline memory::static_allocator_storage<calc_pool_storage_size(
        INPUT_BUFFER_SIZE + OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement))> buffer_storage_ {};
    DMAMEM alignas(8) static inline memory::static_allocator_storage<calc_pool_storage_size(INPUT_BUFFER_SIZE + BUFFER_CHUNK_SIZE)> buffer_storage2_ {};
    using static_pool_t = memory::memory_pool<memory::array_pool, memory::static_allocator>;
    static inline static_pool_t* p_mem_pool_ {};
    static inline static_pool_t* p_mem_pool2_ {};

    static const std::string_view log_prefix_;
    static const std::string_view log_postfix_;

    arduino::SerialIO& io_;
    bool echo_;
    bool viewer_enabled_;
    int error_;
    char* p_input_;
    CircularBuffer<OutBufferElement, OUTPUT_QUEUE_SIZE>* p_output_queue_;
    uint16_t input_task_;
    uint16_t output_task_;
    std::array<char, INPUT_BUFFER_SIZE>* p_input_buffer_;
    std::array<char, INPUT_BUFFER_SIZE + 2>* p_clear_str_;

    /**
     * @brief Worker task implementation that processes incoming data
     * @note Pure virtual method, override for specialized implementations
     */
    virtual void run_input() = 0;

    void run_output();

    FLASHMEM size_t queue_debug_msg(char c, const std::string* p_str, bool block);

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
     * @param[in] io_connection: Reference to SerialIO to use
     * @param[in] enable_echo: character echo mode for console, defaults to false
     */
    FLASHMEM CommInterface(arduino::SerialIO& io_connection, bool enable_echo = false);

    /**
     * @brief Destroy the CommInterface object
     */
    FLASHMEM virtual ~CommInterface();

    virtual void begin(const std::string_view& prefix) override;
    virtual size_t log(char c, bool block) override;
    virtual size_t log(const std::string_view& str, bool block) override;

    /**
     * @return Current character echo mode setting
     */
    auto get_echo() const {
        return echo_;
    }

    auto get_viewer_enabled() const {
        return viewer_enabled_;
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

    FLASHMEM void enable_remoteviewer(bool value) {
        viewer_enabled_ = value;
    }

    void set_color(Color fg, Color bg);

    void set_attribute(Attribute a);

    /**
     * @brief Write a character out to a SerialConnection
     * @param[in] c: Character to write
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(char c, bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Pointer to message as C-string
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const char* str, bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] p_str: Message as pointer to std::string, memory will be released after print
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const std::string* p_str, bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Reference to message as string
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const std::string& str, bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Message as string
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(std::string&& str, bool block);

    /**
     * @brief Write a message out to a SerialConnection
     * @param[in] str: Reference to message as string_view
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(const std::string_view& str, bool block);

    /**
     * @brief Write a number out to SerialConnection
     * @param[in] v: Value
     * @param[in] block: Switch blocking mode
     * @return Number of characters written
     */
    FLASHMEM size_t debug_print(logger::Number auto v, bool block) {
        return debug_print(std::to_string(v), block);
    }

    /**
     * @brief Write an formatted string (like std::printf) out to SerialConnection
     * @tparam BLOCK: Switch blocking mode
     * @param[in] format: Format string
     * @return Number of characters written
     */
    template <bool BLOCK = false>
    FLASHMEM_T size_t debug_printf(const char* format, logger::PrintfArg auto... args) {
        return debug_print(LoggerTarget::string_format(format, args...), BLOCK);
    }

    // FLASHMEM size_t debug_print(const arduino::String& str, bool block);

    /**
     * @brief Wait for any outstanding transmission on the serial connection to complete
     */
    FLASHMEM virtual void flush() override;
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
     * @param[in] io_connection: Reference to SerialIO to use
     * @param[in] parser: Reference to CmdParser to use
     * @param[in] enable_echo: character echo mode for console, defaults to false
     */
    FLASHMEM CommInterfaceCmdParser(arduino::SerialIO& io_connection, CmdParser& parser, bool enable_echo = false);

    /**
     * @brief Destroy the CommInterfaceCmdParser object
     */
    FLASHMEM virtual ~CommInterfaceCmdParser() override;

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     */
    virtual void set_echo(bool value) override;
};

} // namespace ctbot

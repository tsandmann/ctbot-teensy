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
 * @file    comm_interface.cpp
 * @brief   Communication interface classes of ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "comm_interface.h"

#include "cmd_parser.h"
#include "ctbot.h"
#include "scheduler.h"
#include "timer.h"

#include "driver/serial_io.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>


namespace ctbot {

PROGMEM static const char _log_prefix_[] { "<log>" };
PROGMEM static const char _log_postfix_[] { "</log>\r\n" };

DMAMEM alignas(8) static memory::static_allocator_storage<256 * sizeof(void*)> g_buffer_storage;

const std::string_view CommInterface::log_prefix_ { _log_prefix_ };
const std::string_view CommInterface::log_postfix_ { _log_postfix_ };

FLASHMEM CommInterface::CommInterface(arduino::SerialIO& io_connection, bool enable_echo)
    : io_ { io_connection }, echo_ { enable_echo }, viewer_enabled_ { false }, error_ {}, p_input_ {}, p_output_queue_ {}, p_input_buffer_ {}, p_clear_str_ {} {
    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface()\r\n"));
    }

    static_assert(calc_pool_storage_size(INPUT_BUFFER_SIZE + OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement) + INPUT_BUFFER_SIZE + BUFFER_CHUNK_SIZE)
            <= sizeof(g_buffer_storage),
        "CommInterface::CommInterface(): g_buffer_storage too small");

    p_mem_pool_ = new static_pool_t { BUFFER_CHUNK_SIZE,
        calc_pool_storage_size(INPUT_BUFFER_SIZE + OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement) + INPUT_BUFFER_SIZE + BUFFER_CHUNK_SIZE), g_buffer_storage };
    configASSERT(p_mem_pool_);
    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface(): p_mem_pool_ created\r\n"));
    }

    p_output_queue_ = new CircularBuffer<OutBufferElement, OUTPUT_QUEUE_SIZE> { p_mem_pool_->try_allocate_array(
        OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement) / BUFFER_CHUNK_SIZE) };
    configASSERT(p_output_queue_);
    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface(): p_output_queue_ created\r\n"));
    }

    auto ptr { p_mem_pool_->try_allocate_array(INPUT_BUFFER_SIZE / BUFFER_CHUNK_SIZE) };
    configASSERT(ptr);
    if (ptr) {
        p_input_buffer_ = new (ptr) std::array<char, INPUT_BUFFER_SIZE>;
        static_assert(sizeof(*p_input_buffer_) == sizeof(*p_input_buffer_) / BUFFER_CHUNK_SIZE * BUFFER_CHUNK_SIZE);
        configASSERT(p_input_buffer_);

        p_input_buffer_->fill(0);
        p_input_ = p_input_buffer_->begin();

        input_task_ = CtBot::get_instance().get_scheduler()->task_add(
            PSTR("commIN"), INPUT_TASK_PERIOD_MS, INPUT_TASK_PRIORITY, INPUT_TASK_STACK_SIZE, [this]() { run_input(); });
    }
    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface(): input_task_ created\r\n"));
    }

    auto ptr2 { p_mem_pool_->try_allocate_array((sizeof(*p_clear_str_) + BUFFER_CHUNK_SIZE - 1) / BUFFER_CHUNK_SIZE) };
    configASSERT(ptr2);
    if (ptr2) {
        p_clear_str_ = new (ptr2) std::array<char, INPUT_BUFFER_SIZE + 2>;
        configASSERT(p_clear_str_);

        *p_clear_str_->begin() = '\r';
        std::memset(&(*p_clear_str_)[1], ' ', INPUT_BUFFER_SIZE);
        *p_clear_str_->rbegin() = '\r';
    }
    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface(): p_clear_str_ created\r\n"));
    }

    output_task_ = CtBot::get_instance().get_scheduler()->task_add(
        PSTR("commOUT"), OUTPUT_TASK_PERIOD_MS, OUTPUT_TASK_PRIORITY, OUTPUT_TASK_STACK_SIZE, [this]() { run_output(); });
    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface(): output_task_ created\r\n"));
    }

    if constexpr (DEBUG_) {
        io_.write(PSTR("CommInterface::CommInterface() done.\r\n"));
        io_.flush();
        io_.flush_direct();
        io_.clear();
    }
}

FLASHMEM CommInterface::~CommInterface() {
    auto schdl { CtBot::get_instance().get_scheduler() };
    schdl->task_remove(input_task_);
    schdl->task_remove(output_task_);
}

FLASHMEM size_t CommInterface::queue_debug_msg(char c, const std::string* p_str, bool block) {
    const auto ret { p_str ? p_str->length() : 1 };
    const OutBufferElement element { p_str, c };

    if (block) {
        p_output_queue_->push(std::move(element));
    } else {
        if (!p_output_queue_->try_push(std::move(element))) {
            return 0;
        }
    }
    return ret;
}

void CommInterface::begin(const std::string_view& prefix) {
    log(prefix, true);
}

size_t CommInterface::log(char c, bool block) {
    size_t ret;

    if (viewer_enabled_) {
        std::string data;
        data.append(log_prefix_);
        data.push_back(c);
        data.append(log_postfix_);
        ret = debug_print(data, block);
    } else {
        ret = debug_print(c, block);
    }

    return ret;
}

size_t CommInterface::log(const std::string_view& str, bool block) {
    size_t ret;

    if (viewer_enabled_) {
        std::string data;
        data.append(log_prefix_);
        data.append(str);
        data.append(log_postfix_);
        ret = debug_print(std::move(data), block);
    } else {
        ret = debug_print(str, block);
    }

    return ret;
}

void CommInterface::set_color(Color fg, Color bg) {
    debug_printf<true>(PSTR("\x1b[%u;%um"), static_cast<uint16_t>(fg) + 30, static_cast<uint16_t>(bg) + 40);
}

void CommInterface::set_attribute(Attribute a) {
    debug_printf<true>(PSTR("\x1b[%um"), static_cast<uint16_t>(a));
}

FLASHMEM size_t CommInterface::debug_print(char c, bool block) {
    return queue_debug_msg(c, nullptr, block);
}

FLASHMEM size_t CommInterface::debug_print(const char* str, bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(const std::string* p_str, bool block) {
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(const std::string& str, bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(std::string&& str, bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(const std::string_view& str, bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

// FLASHMEM size_t CommInterface::debug_print(const arduino::String& str, bool block) {
//     auto p_str { new std::string(str.c_str(), str.length()) };
//     return queue_debug_msg('\0', p_str, block);
// }

FLASHMEM void CommInterface::flush() {
    using namespace std::chrono_literals;
    while (p_output_queue_->size()) {
        std::this_thread::sleep_for(1ms);
    }
}

void CommInterface::run_output() {
    using namespace std::chrono_literals;

    OutBufferElement element;
    while (p_output_queue_->try_pop(element)) {
        if (element.p_str_) {
            size_t written {};
            while (true) { // TODO: timeout?
                written += io_.write(element.p_str_->c_str() + written, element.p_str_->length() - written, true);
                if (written >= element.p_str_->length()) {
                    break;
                }
                std::this_thread::sleep_for(1ms);
            }
            delete element.p_str_;
        } else {
            io_.write(&element.character_, sizeof(element.character_), true);
        }
    }
}

CommInterfaceCmdParser::CommInterfaceCmdParser(arduino::SerialIO& io_connection, CmdParser& parser, bool enable_echo)
    : CommInterface { io_connection, enable_echo }, cmd_parser_ { parser }, history_view_ {} {
    cmd_parser_.set_echo(enable_echo);
}

CommInterfaceCmdParser::~CommInterfaceCmdParser() = default;

void CommInterfaceCmdParser::set_echo(bool value) {
    echo_ = value;
    cmd_parser_.set_echo(value);
}

void CommInterfaceCmdParser::run_input() {
    while (io_.available()) {
        char c;
        io_.read(&c, 1);

        if (p_input_ >= p_input_buffer_->end()) {
            /* no buffer space left */
            p_input_ = p_input_buffer_->begin();
            error_ = 1;
        }

        if (c == '\n' || c == '\r') {
            if (io_.peek() == '\r' || io_.peek() == '\n' || io_.peek() == 0) {
                char tmp;
                io_.read(&tmp, 1);
            }
            *p_input_ = 0;
            if (echo_) {
                queue_debug_msg('\r', nullptr, false);
                queue_debug_msg('\n', nullptr, false);
            }
            const std::string_view str { p_input_buffer_->begin(), static_cast<size_t>(p_input_ - p_input_buffer_->begin()) };
            cmd_parser_.parse(str, *this);
            p_input_ = p_input_buffer_->begin();
            history_view_ = 0;
            break;
        } else if (c == '\b' || c == 0x7f) {
            /* backspace / DEL */
            if (p_input_ > p_input_buffer_->begin()) {
                --p_input_;
                if (echo_) {
                    queue_debug_msg('\b', nullptr, false);
                    queue_debug_msg(' ', nullptr, false);
                    queue_debug_msg('\b', nullptr, false);
                }
            }
            continue;
        } else if (c == '[' || c == 0x1b) {
            /* ESC */
            if (io_.peek() == 'A') {
                /* UP */
                char tmp;
                io_.read(&tmp, 1);
                const auto cmd { cmd_parser_.get_history(++history_view_) };
                if (cmd) {
                    update_line(cmd.value());
                } else {
                    --history_view_;
                }
                continue;
            } else if (io_.peek() == 'B') {
                /* DOWN */
                char tmp;
                io_.read(&tmp, 1);
                if (history_view_ > 1) {
                    const auto cmd { cmd_parser_.get_history(--history_view_) };
                    if (cmd) {
                        update_line(cmd.value());
                    } else {
                        ++history_view_;
                    }
                } else if (history_view_ == 1) {
                    clear_line();
                    p_input_ = p_input_buffer_->begin();
                }
                continue;
            } else if (c == 0x1b) {
                continue;
            }
        }
        *p_input_++ = c;
        if (echo_) {
            queue_debug_msg(c, nullptr, false);
        }
    }
}

void CommInterfaceCmdParser::clear_line() {
    if (!p_clear_str_) {
        return;
    }

    queue_debug_msg('\0', new std::string { p_clear_str_->cbegin(), p_clear_str_->size() }, true);
}

void CommInterfaceCmdParser::update_line(const std::string_view& line) {
    clear_line();
    debug_print(line, true);

    const size_t n { line.size() > INPUT_BUFFER_SIZE ? INPUT_BUFFER_SIZE : line.size() };
    std::strncpy(p_input_buffer_->begin(), line.data(), n);
    p_input_ = &p_input_buffer_->begin()[n];
}

} // namespace ctbot

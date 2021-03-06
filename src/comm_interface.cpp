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
 * @file    comm_interface.cpp
 * @brief   Communication interface classes of c't-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "comm_interface.h"
#include "ctbot.h"
#include "cmd_parser.h"
#include "serial_connection_teensy.h"
#include "timer.h"
#include "scheduler.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>


namespace ctbot {

decltype(CommInterface::buffer_storage_) CommInterface::buffer_storage_;
CommInterface::static_pool_t CommInterface::mem_pool_ { BUFFER_CHUNK_SIZE, sizeof(buffer_storage_), buffer_storage_ };

CommInterface::CommInterface(SerialConnectionTeensy& io_connection, bool enable_echo)
    : io_ { io_connection }, echo_ { enable_echo }, error_ {}, p_input_ {}, output_queue_ { mem_pool_.try_allocate_array(
                                                                                OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement) / BUFFER_CHUNK_SIZE) } {
    auto ptr { mem_pool_.try_allocate_array(INPUT_BUFFER_SIZE / BUFFER_CHUNK_SIZE) };
    if (ptr) {
        p_input_buffer_ = new (ptr) std::array<char, INPUT_BUFFER_SIZE>;
        static_assert(sizeof(*p_input_buffer_) == INPUT_BUFFER_SIZE / BUFFER_CHUNK_SIZE * BUFFER_CHUNK_SIZE);

        p_input_buffer_->fill(0);
        p_input_ = p_input_buffer_->begin();

        input_task_ = CtBot::get_instance().get_scheduler()->task_add(
            PSTR("commIN"), INPUT_TASK_PERIOD_MS, INPUT_TASK_PRIORITY, INPUT_TASK_STACK_SIZE, [this]() { return run_input(); });
    }

    output_task_ = CtBot::get_instance().get_scheduler()->task_add(
        PSTR("commOUT"), OUTPUT_TASK_PERIOD_MS, OUTPUT_TASK_PRIORITY, OUTPUT_TASK_STACK_SIZE, [this]() { return run_output(); });
}

CommInterface::~CommInterface() {
    auto schdl { CtBot::get_instance().get_scheduler() };
    flush();
    schdl->task_remove(input_task_);
    schdl->task_remove(output_task_);
}

size_t CommInterface::queue_debug_msg(const char c, const std::string* p_str, const bool block) {
    const auto ret { p_str ? p_str->length() : 1 };
    const OutBufferElement element { p_str, c };

    if (block) {
        output_queue_.push(std::move(element));
    } else {
        if (!output_queue_.try_push(std::move(element))) {
            return 0;
        }
    }
    return ret;
}

size_t CommInterface::get_format_size(const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    const auto size { std::vsnprintf(nullptr, 0, format, vl) + 1 };
    va_end(vl);
    return size;
}

std::string* CommInterface::create_formatted_string(const size_t size, const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    auto p_str { new std::string(size, '\0') };
    std::vsnprintf(p_str->data(), size, format, vl);
    va_end(vl);

    return p_str;
}

void CommInterface::set_color(const Color fg, const Color bg) {
    debug_printf<true>(PSTR("\x1b[%u;%um"), static_cast<uint16_t>(fg) + 30, static_cast<uint16_t>(bg) + 40);
}

void CommInterface::set_attribute(const Attribute a) {
    debug_printf<true>(PSTR("\x1b[%um"), static_cast<uint16_t>(a));
}

size_t CommInterface::debug_print(const char* str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

size_t CommInterface::debug_print(const std::string* p_str, const bool block) {
    return queue_debug_msg('\0', p_str, block);
}

size_t CommInterface::debug_print(const std::string& str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

size_t CommInterface::debug_print(std::string&& str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

size_t CommInterface::debug_print(const std::string_view& str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

// size_t CommInterface::debug_print(const arduino::String& str, const bool block) {
//     auto p_str { std::make_unique<std::string>(str.c_str(), str.length()) };
//     return queue_debug_msg('\0', std::move(p_str), block);
// }

void CommInterface::flush() {
    using namespace std::chrono_literals;
    while (output_queue_.size()) {
        std::this_thread::sleep_for(1ms);
    }
}

void CommInterface::run_output() {
    OutBufferElement element;
    while (output_queue_.try_pop(element)) {
        if (element.p_str_) {
            io_.send(element.p_str_->c_str(), element.p_str_->length());
            delete element.p_str_;
        } else {
            io_.send(&element.character_, sizeof(element.character_));
        }
    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(50ms);
}

CommInterfaceCmdParser::CommInterfaceCmdParser(SerialConnectionTeensy& io_connection, CmdParser& parser, bool enable_echo)
    : CommInterface { io_connection, enable_echo }, cmd_parser_ { parser }, history_view_ {} {
    cmd_parser_.set_echo(enable_echo);
}

void CommInterfaceCmdParser::set_echo(bool value) {
    echo_ = value;
    cmd_parser_.set_echo(value);
}

void CommInterfaceCmdParser::run_input() {
    while (io_.available()) {
        char c;
        io_.receive(&c, 1);

        if (p_input_ >= p_input_buffer_->end()) {
            /* no buffer space left */
            p_input_ = p_input_buffer_->begin();
            error_ = 1;
        }

        if (c == '\n' || c == '\r') {
            if (io_.peek() == '\r' || io_.peek() == '\n' || io_.peek() == 0) {
                char tmp;
                io_.receive(&tmp, 1);
            }
            *p_input_ = 0;
            if (echo_) {
                io_.send(PSTR("\r\n"), 2);
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
                    io_.send(PSTR("\b \b"), 3);
                }
            }
            continue;
        } else if (c == '[' || c == 0x1b) {
            /* ESC */
            if (io_.peek() == 'A') {
                /* UP */
                char tmp;
                io_.receive(&tmp, 1);
                auto cmd { cmd_parser_.get_history(++history_view_) };
                if (!cmd.empty()) {
                    update_line(cmd);
                } else {
                    --history_view_;
                }
                continue;
            } else if (io_.peek() == 'B') {
                /* DOWN */
                char tmp;
                io_.receive(&tmp, 1);
                if (history_view_ > 1) {
                    auto cmd { cmd_parser_.get_history(--history_view_) };
                    if (!cmd.empty()) {
                        update_line(cmd);
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
            io_.send(&c, 1);
        }
    }
}

void CommInterfaceCmdParser::clear_line() {
    io_.send("\r", 1);
    for (size_t i { 0 }; i < INPUT_BUFFER_SIZE; ++i) {
        io_.send(" ", 1);
    }
    io_.send("\r", 1);
}

void CommInterfaceCmdParser::update_line(const std::string_view& line) {
    clear_line();
    io_.send(line);

    const size_t n { line.size() > INPUT_BUFFER_SIZE ? INPUT_BUFFER_SIZE : line.size() };
    std::strncpy(p_input_buffer_->begin(), line.data(), n);
    p_input_ = &p_input_buffer_->begin()[n];
}

} // namespace ctbot

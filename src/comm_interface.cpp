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

decltype(CommInterface::buffer_storage_) CommInterface::buffer_storage_;
CommInterface::static_pool_t CommInterface::mem_pool_ { BUFFER_CHUNK_SIZE, sizeof(buffer_storage_), buffer_storage_ };

PROGMEM static const char _log_prefix_[] { "<log>\r\n" };
PROGMEM static const char _log_postfix_[] { "</log>\r\n" };

const std::string_view CommInterface::log_prefix_ { _log_prefix_ };
const std::string_view CommInterface::log_postfix_ { _log_postfix_ };

FLASHMEM CommInterface::CommInterface(arduino::SerialIO& io_connection, bool enable_echo)
    : io_ { io_connection }, echo_ { enable_echo }, viewer_enabled_ { false }, error_ {}, p_input_ {}, output_queue_ {
          mem_pool_.try_allocate_array(OUTPUT_QUEUE_SIZE * sizeof(OutBufferElement) / BUFFER_CHUNK_SIZE)
      } {
    auto ptr { mem_pool_.try_allocate_array(INPUT_BUFFER_SIZE / BUFFER_CHUNK_SIZE) };
    if (ptr) {
        p_input_buffer_ = new (ptr) std::array<char, INPUT_BUFFER_SIZE>;
        static_assert(sizeof(*p_input_buffer_) == INPUT_BUFFER_SIZE / BUFFER_CHUNK_SIZE * BUFFER_CHUNK_SIZE);

        p_input_buffer_->fill(0);
        p_input_ = p_input_buffer_->begin();

        input_task_ = CtBot::get_instance().get_scheduler()->task_add(
            PSTR("commIN"), INPUT_TASK_PERIOD_MS, INPUT_TASK_PRIORITY, INPUT_TASK_STACK_SIZE, [this]() { run_input(); });
    }

    output_task_ = CtBot::get_instance().get_scheduler()->task_add(
        PSTR("commOUT"), OUTPUT_TASK_PERIOD_MS, OUTPUT_TASK_PRIORITY, OUTPUT_TASK_STACK_SIZE, [this]() { run_output(); });
}

FLASHMEM CommInterface::~CommInterface() {
    auto schdl { CtBot::get_instance().get_scheduler() };
    flush();
    schdl->task_remove(input_task_);
    schdl->task_remove(output_task_);
}

FLASHMEM size_t CommInterface::queue_debug_msg(const char c, const std::string* p_str, const bool block) {
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

void CommInterface::begin(const std::string_view&) const {}

size_t CommInterface::log(const char c, const bool block) {
    if (viewer_enabled_) { // FIXME: prefix/postfix is not atomic
        debug_print(log_prefix_, block);
    }
    const auto ret { debug_print(c, block) };
    if (viewer_enabled_) {
        debug_print(log_postfix_, block);
    }

    return ret;
}

size_t CommInterface::log(const std::string_view& str, const bool block) {
    if (viewer_enabled_) { // FIXME: prefix/postfix is not atomic
        debug_print(log_prefix_, block);
    }
    const auto ret { debug_print(str, block) };
    if (viewer_enabled_) {
        debug_print(log_postfix_, block);
    }

    return ret;
}

void CommInterface::set_color(const Color fg, const Color bg) {
    debug_printf<true>(PSTR("\x1b[%u;%um"), static_cast<uint16_t>(fg) + 30, static_cast<uint16_t>(bg) + 40);
}

void CommInterface::set_attribute(const Attribute a) {
    debug_printf<true>(PSTR("\x1b[%um"), static_cast<uint16_t>(a));
}

FLASHMEM size_t CommInterface::debug_print(const char c, const bool block) {
    return queue_debug_msg(c, nullptr, block);
}

FLASHMEM size_t CommInterface::debug_print(const char* str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(const std::string* p_str, const bool block) {
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(const std::string& str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(std::string&& str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

FLASHMEM size_t CommInterface::debug_print(const std::string_view& str, const bool block) {
    auto p_str { new std::string(str) };
    return queue_debug_msg('\0', p_str, block);
}

// FLASHMEM size_t CommInterface::debug_print(const arduino::String& str, const bool block) {
//     auto p_str { new std::string(str.c_str(), str.length()) };
//     return queue_debug_msg('\0', p_str, block);
// }

FLASHMEM void CommInterface::flush() {
    using namespace std::chrono_literals;
    while (output_queue_.size()) {
        std::this_thread::sleep_for(1ms);
    }
}

void CommInterface::run_output() {
    using namespace std::chrono_literals;

    OutBufferElement element;
    while (output_queue_.try_pop(element)) {
        if (element.p_str_) {
            size_t written {};
            while (true) { // FIXME: timeout?
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
                io_.read(&tmp, 1);
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
            queue_debug_msg(c, nullptr, false);
        }
    }
}

void CommInterfaceCmdParser::clear_line() {
    queue_debug_msg('\r', nullptr, false);
    for (size_t i { 0 }; i < INPUT_BUFFER_SIZE; ++i) {
        queue_debug_msg(' ', nullptr, false);
    }
    queue_debug_msg('\r', nullptr, false);
}

void CommInterfaceCmdParser::update_line(const std::string_view& line) {
    clear_line();
    debug_print(line, false);

    const size_t n { line.size() > INPUT_BUFFER_SIZE ? INPUT_BUFFER_SIZE : line.size() };
    std::strncpy(p_input_buffer_->begin(), line.data(), n);
    p_input_ = &p_input_buffer_->begin()[n];
}

} // namespace ctbot

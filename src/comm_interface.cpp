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
#include "leds.h"

#include "FreeRTOS.h"
#include "queue.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>


namespace ctbot {

CommInterface::CommInterface(SerialConnectionTeensy& io_connection, bool enable_echo)
    : io_ { io_connection }, echo_ { enable_echo }, error_ { 0 }, p_input_ { input_buffer_ }, output_queue_ { ::xQueueCreate(
                                                                                                  OUTPUT_QUEUE_SIZE, sizeof(OutBufferElement)) } {
    configASSERT(output_queue_);

    CtBot::get_instance().get_scheduler()->task_add(
        "commIN", INPUT_TASK_PERIOD_MS, INPUT_TASK_PRIORITY, INPUT_TASK_STACK_SIZE, [this]() { return run_input(); });
    CtBot::get_instance().get_scheduler()->task_add(
        "commOUT", OUTPUT_TASK_PERIOD_MS, OUTPUT_TASK_PRIORITY, OUTPUT_TASK_STACK_SIZE, [this]() { return run_output(); });
}

size_t CommInterface::queue_debug_msg(const char c, std::string* p_str, const bool block) const {
    OutBufferElement element { c, p_str };
    if (::xQueueSendToBack(output_queue_, &element, block ? portMAX_DELAY : 0)) {
        return p_str ? p_str->length() : 1;
    } else {
        return 0;
    }
}

size_t CommInterface::get_format_size(const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    const auto size { std::vsnprintf(nullptr, 0, format, vl) };
    va_end(vl);
    return size;
}

std::unique_ptr<char> CommInterface::create_formatted_string(const size_t size, const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    std::unique_ptr<char> str { new char[size] };
    std::vsnprintf(str.get(), size, format, vl);
    va_end(vl);
    return str;
}

void CommInterface::set_color(const Color fg, const Color bg) {
    debug_printf<true>("\x1b[%u;%um", static_cast<uint16_t>(fg) + 30, static_cast<uint16_t>(bg) + 40);
}

void CommInterface::set_attribute(const Attribute a) {
    debug_printf<true>("\x1b[%um", static_cast<uint16_t>(a));
}

size_t CommInterface::debug_print(const char* str, const bool block) const {
    return queue_debug_msg('\0', new std::string(str), block);
}

size_t CommInterface::debug_print(const std::string& str, const bool block) const {
    return queue_debug_msg('\0', new std::string(str), block);
}

size_t CommInterface::debug_print(std::string&& str, const bool block) const {
    return queue_debug_msg('\0', new std::string(str), block);
}

void CommInterface::flush() {
    while (::uxQueueMessagesWaiting(output_queue_)) {
        Timer::delay_ms(1);
    }
}

void CommInterface::run_output() {
    OutBufferElement element;
    while (::xQueueReceive(output_queue_, &element, portMAX_DELAY)) {
        if (element.p_str_) {
            io_.send(element.p_str_->c_str(), element.p_str_->length());
            delete element.p_str_;
        } else {
            io_.send(&element.character_, sizeof(element.character_));
        }
    }
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

        if (p_input_ >= &input_buffer_[sizeof(input_buffer_)]) {
            /* no buffer space left */
            p_input_ = input_buffer_;
            error_ = 1;
        }

        if (c == '\n' || c == '\r') {
            if (io_.peek() == '\r' || io_.peek() == '\n' || io_.peek() == 0) {
                char tmp;
                io_.receive(&tmp, 1);
            }
            *p_input_ = 0;
            if (echo_) {
                io_.send("\r\n", 2);
            }
            cmd_parser_.parse(input_buffer_, *this);
            p_input_ = input_buffer_;
            history_view_ = 0;
            break;
        } else if (c == '\b' || c == 0x7f) {
            /* backspace / DEL */
            if (p_input_ > input_buffer_) {
                --p_input_;
                if (echo_) {
                    const char tmp[] { "\b \b" };
                    io_.send(tmp, sizeof(tmp) - 1);
                }
            }
            continue;
        } else if (c == '[' || c == 0x1b) {
            /* ESC */
            if (io_.peek() == 'A') {
                /* UP */
                char tmp;
                io_.receive(&tmp, 1);
                auto p_cmd { cmd_parser_.get_history(++history_view_) };
                if (p_cmd) {
                    update_line(*p_cmd);
                } else {
                    --history_view_;
                }
                continue;
            } else if (io_.peek() == 'B') {
                /* DOWN */
                char tmp;
                io_.receive(&tmp, 1);
                if (history_view_ > 1) {
                    auto p_cmd { cmd_parser_.get_history(--history_view_) };
                    if (p_cmd) {
                        update_line(*p_cmd);
                    } else {
                        ++history_view_;
                    }
                } else if (history_view_ == 1) {
                    clear_line();
                    p_input_ = input_buffer_;
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

void CommInterfaceCmdParser::update_line(const std::string& line) {
    clear_line();
    io_.send(line.c_str(), line.size());

    const size_t n { line.size() > INPUT_BUFFER_SIZE ? INPUT_BUFFER_SIZE : line.size() };
    std::strncpy(input_buffer_, line.c_str(), n);
    p_input_ = &input_buffer_[n];
}

} // namespace ctbot

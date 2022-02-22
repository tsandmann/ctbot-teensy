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
 * @file    ctbot_task.cpp
 * @brief   Task implementation
 * @author  Timo Sandmann
 * @date    28.07.2018
 */

#include "ctbot_task.h"

#include "comm_interface.h"
#include "scheduler.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"

#include <algorithm>


namespace ctbot {

Task::Task(
    Scheduler& scheduler, const uint16_t id, const std::string_view& name, const bool external, const uint16_t period, const uint8_t priority, func_t&& func)
    : id_ { id }, period_ { period }, priority_ { priority }, state_ { 1 }, func_ { std::move(func) }, std_thread_ {}, external_ { external }, handle_ {},
      scheduler_ { scheduler }, name_ { std::string { name } } {}

void Task::print(CommInterface& comm) const {
    comm.debug_printf<true>(PP_ARGS(" \"{s}\":\t{#x}\t", name_.c_str(), id_));
    comm.debug_print(state_ == 1 ? PSTR("  ACTIVE") : (state_ == 2 ? PSTR(" BLOCKED") : PSTR("FINISHED")), true);
    comm.debug_printf<true>(PP_ARGS("\tprio: {} \t", get_priority()));
    if (period_) {
        comm.debug_printf<true>(PP_ARGS("{} ms", period_));
    }
    comm.debug_printf<true>(PP_ARGS("\tstack free: {} byte", scheduler_.get_free_stack(id_)));
    comm.debug_printf<true>(
        PP_ARGS("\thandle: {#x}\r\n", std_thread_ ? free_rtos_std::gthr_freertos::get_freertos_handle(handle_.p_thread) : handle_.p_freertos_handle));
}

Task::~Task() {
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("Task::~Task() for \""));
        ::serialport_puts(name_.c_str());
        ::serialport_puts(PSTR("\"\r\n"));
    }

    state_ = 0;
    if (std_thread_) {
        if (handle_.p_thread && !external_) {
            if (DEBUG_LEVEL_ > 3) {
                ::serialport_puts(PSTR("Task::~Task(): task is std::thread.\r\n"));
            }
            if (handle_.p_thread->joinable()) {
                if (name_ != PSTR("main")) {
                    if (DEBUG_LEVEL_ > 3) {
                        ::serialport_puts(PSTR("Task::~Task(): calling join()...\r\n"));
                    }
                    handle_.p_thread->join();
                    if (DEBUG_LEVEL_ > 3) {
                        ::serialport_puts(PSTR("Task::~Task(): join() done.\r\n"));
                    }
                } else {
                    handle_.p_thread->detach();
                    if (DEBUG_LEVEL_ > 3) {
                        ::serialport_puts(PSTR("Task::~Task(): detach() done.\r\n"));
                    }
                }
            } else {
                if (DEBUG_LEVEL_ > 3) {
                    ::serialport_puts(PSTR("Task::~Task(): not joinable.\r\n"));
                }
            }
            delete handle_.p_thread;
            if (DEBUG_LEVEL_ > 2) {
                ::serialport_puts(PSTR("Task::~Task(): thread deleted.\r\n"));
            }
        }
    } else {
        if (DEBUG_LEVEL_ > 3) {
            ::serialport_puts(PSTR("Task::~Task(): task is pure FreeRTOS task.\r\n"));
        }
        if (handle_.p_freertos_handle && !external_) {
            ::vTaskDelete(handle_.p_freertos_handle);
            if (DEBUG_LEVEL_ > 2) {
                ::serialport_puts(PSTR("Task::~Task(): task handle deleted.\r\n"));
            }
        }
    }
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("Task::~Task() done.\r\n"));
    }
}

bool Task::resume() {
    return scheduler_.task_resume(id_);
}

TaskHandle_t Task::get_handle() const {
    if (std_thread_) {
        return handle_.p_thread->native_handle().get_native_handle();
    }

    return handle_.p_freertos_handle;
}

} // namespace ctbot

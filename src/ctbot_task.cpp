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
 * @file    ctbot_task.cpp
 * @brief   Task implementation
 * @author  Timo Sandmann
 * @date    28.07.2018
 */

#include "ctbot_task.h"
#include "scheduler.h"
#include "comm_interface.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"
#include "portable/teensy.h"

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
    comm.debug_printf<true>(PP_ARGS("\tstack free: {} byte\r\n", scheduler_.get_free_stack(id_)));
}

Task::~Task() {
    arduino::Serial.print(PSTR("Task::~Task() for \""));
    arduino::Serial.print(name_.c_str());
    arduino::Serial.println('\"');

    state_ = 0;
    if (std_thread_) {
        if (handle_.p_thread && !external_) {
            ::serial_puts(PSTR("Task::~Task(): task is std::thread."));
            if (handle_.p_thread->joinable()) {
                if (name_ != PSTR("main")) {
                    ::serial_puts(PSTR("Task::~Task(): calling join()..."));
                    handle_.p_thread->join();
                    ::serial_puts(("Task::~Task(): join() done."));
                } else {
                    handle_.p_thread->detach();
                    ::serial_puts(PSTR("Task::~Task(): detach() done."));
                }
            } else {
                ::serial_puts(PSTR("Task::~Task(): not joinable."));
            }
            delete handle_.p_thread;
            ::serial_puts(PSTR("Task::~Task(): thread deleted."));
        }
    } else {
        ::serial_puts(PSTR("Task::~Task(): task is pure FreeRTOS task."));
        if (handle_.p_freertos_handle && !external_) {
            ::vTaskDelete(handle_.p_freertos_handle);
        }
    }
    ::serial_puts(PSTR("Task::~Task() done."));
}

bool Task::resume() {
    return scheduler_.task_resume(id_);
}

} // namespace ctbot

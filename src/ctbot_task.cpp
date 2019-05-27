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
#include "arduino_fixed.h"
#include "FreeRTOS.h"
#include "task.h"
#include <algorithm>


namespace ctbot {

Task::Task(Scheduler& scheduler, const uint16_t id, const std::string& name, const uint16_t period, const uint8_t priority, func_t&& func)
    : id_ { id }, period_ { period }, priority_ { priority }, state_ { 1 }, func_ { std::move(func) }, std_thread_ { false }, handle_ {},
      scheduler_ { scheduler }, name_ { name } {}

void Task::print(CommInterface& comm) const {
    comm.debug_printf<true>(PP_ARGS(" \"{s}\":\t{#x}\t", name_.c_str(), id_));
    comm.debug_print(state_ == 1 ? "  ACTIVE" : (state_ == 2 ? " BLOCKED" : "FINISHED"), true);
    comm.debug_printf<true>(PP_ARGS("\tprio: {} \t", get_priority()));
    if (period_) {
        comm.debug_printf<true>(PP_ARGS("{} ms", period_));
    }
    comm.debug_printf<true>(PP_ARGS("\tstack free: {} byte\r\n", scheduler_.get_free_stack(id_)));
}

Task::~Task() {
    state_ = 0;
    if (std_thread_) {
        handle_.p_thread->detach();
        delete handle_.p_thread;
    } else {
        ::vTaskDelete(handle_.p_freertos_handle);
    }
}

bool Task::resume() {
    return scheduler_.task_resume(id_);
}

} // namespace ctbot

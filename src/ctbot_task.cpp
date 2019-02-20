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
#include "condition.h"
#include "scheduler.h"
#include "comm_interface.h"

#include "pprintpp.hpp"
#include "arduino_fixed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <algorithm>


namespace ctbot {

Task::Task(Scheduler& scheduler, const uint16_t id, const std::string& name, const uint16_t period, func_t&& func)
    : id_ { id }, period_ { period }, state_ { 1 }, func_ { std::move(func) }, handle_ { nullptr }, scheduler_ { scheduler }, name_ { name } {}

void Task::print(CommInterface& comm) const {
    comm.debug_printf<true>(PP_ARGS(" \"{s}\":\t{#x}\t", name_.c_str(), id_));
    comm.debug_print(state_ == 1 ? "  ACTIVE" : (state_ == 2 ? " BLOCKED" : "FINISHED"), true);
    comm.debug_printf<true>(PP_ARGS("\tprio: {} \t", get_priority()));
    if (period_) {
        comm.debug_printf<true>(PP_ARGS("{} ms", period_));
    }
#if INCLUDE_uxTaskGetStackHighWaterMark == 1
    const size_t stack_free { ::uxTaskGetStackHighWaterMark(handle_) * sizeof(StackType_t) };
#else
    const size_t stack_free { 0xffffff };
#endif

    comm.debug_printf<true>(PP_ARGS("\tstack free: {} byte\r\n", stack_free));
}

Task::~Task() {
    state_ = 0;
    if (handle_) {
        ::vTaskDelete(handle_);
    }
}

bool Task::wait_for(Condition& cond) {
    if (!scheduler_.task_wait_for(id_, cond)) {
        return false;
    }

    while (cond.task_waiting(this)) {
        scheduler_.task_suspend(id_);
    }

    return true;
}

bool Task::resume() {
    return scheduler_.task_resume(id_);
}

uint16_t Task::get_priority() const {
    return ::uxTaskPriorityGet(handle_);
}

} // namespace ctbot

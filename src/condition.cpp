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
 * @file    condition.cpp
 * @brief   Condition variable implementation
 * @author  Timo Sandmann
 * @date    28.07.2018
 */

#include "condition.h"
#include "scheduler.h"

#include <algorithm>


namespace ctbot {

void Condition::add_task(Task* p_task) {
    if (!p_task) {
        return;
    }

    Scheduler::enter_critical_section();
    wait_list_.push_back(p_task);
    Scheduler::exit_critical_section();
}

void Condition::remove_task(Task* p_task) {
    if (!p_task) {
        return;
    }

    Scheduler::enter_critical_section();
    wait_list_.remove(p_task);
    Scheduler::exit_critical_section();
}

bool Condition::notify() {
    Scheduler::enter_critical_section();
    if (wait_list_.empty()) {
        Scheduler::exit_critical_section();
        return false;
    }
    Task* p_task { wait_list_.front() };
    wait_list_.pop_front();
    Scheduler::exit_critical_section();

    return p_task->resume();
}

bool Condition::notify_all() {
    bool ret { true };
    Scheduler::enter_critical_section();
    if (wait_list_.empty()) {
        Scheduler::exit_critical_section();
        return false;
    }
    /* resume all before releasing scheduler lock, therefore not using a mutex here */
    for (auto p_task : wait_list_) {
        ret &= p_task->resume();
    }
    wait_list_.clear();
    Scheduler::exit_critical_section();

    return ret;
}

bool Condition::task_waiting(const Task* p_task) const {
    if (!p_task) {
        return false;
    }

    bool ret;
    Scheduler::enter_critical_section();
    /* we don't expect wait_list_ to be big, so blocking here should be okay */
    ret = std::find(wait_list_.begin(), wait_list_.end(), p_task) != wait_list_.end();
    Scheduler::exit_critical_section();
    return ret;
}

} // namespace ctbot

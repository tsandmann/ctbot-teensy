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
 * @file    scheduler.cpp
 * @brief   Task scheduling
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "scheduler.h"
#include "condition.h"
#include "ctbot.h"
#include "timer.h"
#include "comm_interface.h"

#include "pprintpp.hpp"
#include "arduino_fixed.h"


namespace ctbot {

Scheduler::Scheduler() : next_id_ {} {}

void Scheduler::run() {
    // arduino::Serial.println("Scheduler::run() entered.");
    while (!task_queue_.empty()) {
        Task* p_task { task_queue_.top() };
        task_queue_.pop();

        // arduino::Serial.print("Scheduler::run(): next_runtime=");
        // arduino::Serial.print(p_task->next_runtime_, 10);
        // arduino::Serial.println("ms");

        const uint32_t next_runtime { p_task->next_runtime_ };
        uint32_t now { Timer::get_ms() };

        // arduino::Serial.print("Scheduler::run(): now=");
        // arduino::Serial.print(now, 10);
        // arduino::Serial.println("ms");

        while (next_runtime > now) {
            // FIXME: sleep?
            now = Timer::get_ms();
        }

        // arduino::Serial.print("Scheduler::run(): now=");
        // arduino::Serial.print(now, 10);
        // arduino::Serial.println("ms");

        if (p_task->state_ == 1) {
            // arduino::Serial.print("Scheduler::run(): executing task \"");
            // arduino::Serial.print(p_task->name_.c_str());
            // arduino::Serial.println("\"");

            p_task->func_();
        }
        p_task->next_runtime_ = now + p_task->period_;
        task_queue_.push(p_task);
    }
}

void Scheduler::stop() {
    while (true) {
        // FIXME: sleep?
    }
}

uint16_t Scheduler::task_add(const std::string& name, const uint16_t period, const uint8_t priority, const uint32_t, Task::func_t&& func) {
    Task* p_task { new Task(*this, next_id_, name, period, std::move(func)) };
    task_queue_.push(p_task);
    tasks_[p_task->id_] = p_task;
    return next_id_++;
}

bool Scheduler::task_remove(const uint16_t task_id) {
    Task* p_task { tasks_.at(task_id) };
    tasks_.erase(task_id);
    delete p_task;

    return true;
}

uint16_t Scheduler::task_get(const std::string& name) const {
    for (const auto& t : tasks_) {
        if (t.second->name_ == name) {
            return t.first;
        }
    }

    return 0xffff;
}

Task* Scheduler::task_get(const uint16_t id) const {
    return tasks_.at(id);
}

bool Scheduler::task_suspend(const uint16_t id) {
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 2;

    return true;
}

bool Scheduler::task_resume(const uint16_t id) {
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 1;

    return true;
}

bool Scheduler::task_wait_for(const uint16_t id, Condition& cond) {
    cond.add_task(task_get(id));

    if (!task_suspend(id)) {
        cond.remove_task(task_get(id));
        return false;
    }

    return true;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (const auto& p_task : tasks_) {
        p_task.second->print(comm);
    }
}

void Scheduler::print_ram_usage(CommInterface& comm) const {}
} // namespace ctbot

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
#include "timer.h"
#include "comm_interface.h"


namespace ctbot {

Scheduler::Task::Task(const uint16_t id, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data)
    : id_ { id }, period_ { period }, next_runtime_ { 0 }, active_ { true }, func_ { std::move(func) }, func_data_ { std::move(func_data) } {}

Scheduler::Task::Task(const uint16_t id, const uint16_t period, const uint32_t next_run, task_func_t&& func, task_func_data_t&& func_data)
    : id_ { id }, period_ { period }, next_runtime_ { next_run }, active_ { true }, func_ { std::move(func) }, func_data_ { std::move(func_data) } {}


void Scheduler::Task::print(CommInterface& comm) const {
    comm.debug_print("0x");
    comm.debug_print(id_, PrintBase::HEX);
    comm.debug_print('\t');
    comm.debug_print(active_ ? "  ACTIVE" : "INACTIVE");
    comm.debug_print('\t');
    comm.debug_print(period_, PrintBase::DEC);
    comm.debug_print(" ms\n");
}


void Scheduler::run() {
    while (running_ && (!task_queue_.empty())) {
        const Task& task { task_queue_.top() };
        // std::cout << "Scheduler::run(): task.next_runtime=" << task.next_runtime_ << " ms.\n";
        const uint32_t next_runtime { task.next_runtime_ };
        uint32_t now { Timer::get_ms() };
        // std::cout << "Scheduler::run(): now=" << now << " ms \n";
        while (next_runtime > now) {
            // FIXME: sleep
            now = Timer::get_ms();
        }
        // std::cout << "Scheduler::run(): now=" << now << " ticks \n";
        if (task.active_) {
            // std::cout << "Scheduler::run(): executing function with period " << task.period_ << " ms.\n";
            task.func_(task.func_data_);
            // std::cout << "Scheduler::run(): function with period " << task.period_ << " ms done.\n";
        }
        Task new_task { task };
        new_task.next_runtime_ = now + new_task.period_;
        task_queue_.pop();
        task_queue_.push(new_task);
    }
}

uint16_t Scheduler::task_add(const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data) {
    Task task { next_id_, period, Timer::get_ms(), std::move(func), std::move(func_data) };
    task_queue_.push(task);
    task_names_.push_back(name);

    // std::cout << "Scheduler::task_add(): task \"" << task_names_[next_id_] << "\" with id 0x" << std::hex << task.id_ << " and period of "
    //     << std::dec << task.period_ << " ms added.\n";
    return next_id_++;
}

uint16_t Scheduler::task_get(const std::string& name) const {
    for (uint16_t i { 0U }; i < next_id_; ++i) {
        if (task_names_[i] == name) {
            return i;
        }
    }

    return 0xffff;
}

bool Scheduler::task_suspend(const uint16_t id) {
    // std::cout << "Scheduler::task_suspend(0x" << std::hex << id << std::dec << ")\n";
    bool res { false };
    for (auto& t : task_vector_) {
        if (t.id_ == id) {
            t.active_ = false;
            res = true;
            break;
        }
    }

    return res;
}

bool Scheduler::task_resume(const uint16_t id) {
    // std::cout << "Scheduler::task_resume(0x" << std::hex << id << std::dec << ")\n";
    bool res { false };
    for (auto& t : task_vector_) {
        if (t.id_ == id) {
            t.active_ = true;
            res = true;
            break;
        }
    }

    return res;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (uint16_t i { 0U }; i < next_id_; ++i) {
        comm.debug_print(" \"");
        comm.debug_print(task_names_[task_vector_[i].id_]);
        comm.debug_print("\":\t");
        task_vector_[i].print(comm);
    }
}

} // namespace ctbot

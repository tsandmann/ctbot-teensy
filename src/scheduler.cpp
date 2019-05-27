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
#include "ctbot.h"
#include "timer.h"
#include "comm_interface.h"

#include "pprintpp.hpp"
#include "arduino_fixed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portable/teensy.h"


namespace ctbot {

Scheduler::Scheduler() : next_id_ {} {
    Task* p_idle_task { new Task(*this, next_id_++, configIDLE_TASK_NAME, 0, ::uxTaskPriorityGet(::xTaskGetIdleTaskHandle()), nullptr) };
    p_idle_task->std_thread_ = false;
    p_idle_task->handle_.p_freertos_handle = ::xTaskGetIdleTaskHandle();

    ::vTaskSuspendAll();
    tasks_[0] = p_idle_task;
    ::xTaskResumeAll();
}

Scheduler::~Scheduler() {
    // arduino::Serial.println("Scheduler::~Scheduler() entered.");
    enter_critical_section();
    for (auto& t : tasks_) {
        if (t.second->name_ != "main") {
            // arduino::Serial.print("Scheduler::~Scheduler(): task 0x");
            // arduino::Serial.print(t.second->id_, 16);
            // arduino::Serial.print(" @ 0x");
            // arduino::Serial.print(reinterpret_cast<uintptr_t>(t.second), 16);
            // arduino::Serial.print(" will be blocked...");
            t.second->state_ = 0;
            // arduino::Serial.println(" done.");
        }
    }
    // arduino::Serial.println("Scheduler::~Scheduler(): all tasks blocked.");

    for (auto& t : tasks_) {
        if (t.second->name_ != "main") {
            // arduino::Serial.print("Scheduler::~Scheduler(): task \"");
            // arduino::Serial.print(t.second->name_.c_str());
            // arduino::Serial.print("\" will be deleted...");
            delete t.second;
            // arduino::Serial.println(" done.");
        }
    }
    exit_critical_section();
    // serial_puts("Scheduler::~Scheduler(): all tasks deleted.");
}

void Scheduler::stop() {
    ::vTaskEndScheduler();
}

size_t Scheduler::get_free_stack() {
    return ::uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
}

size_t Scheduler::get_free_stack(const uint16_t id) {
    auto p_task { task_get(id) };
    return ::uxTaskGetStackHighWaterMark(
               p_task->std_thread_ ? free_rtos_std::gthr_freertos::get_freertos_handle(p_task->handle_.p_thread) : p_task->handle_.p_freertos_handle)
        * sizeof(StackType_t);
}

uint16_t Scheduler::task_add(const std::string& name, const uint16_t period, const uint8_t priority, const uint32_t stack_size, Task::func_t&& func) {
    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = new Task(*this, next_id_++, name, period, priority, std::move(func));
        tasks_[p_task->id_] = p_task;
    }

    p_task->std_thread_ = true;
    const auto last { free_rtos_std::gthr_freertos::set_next_stacksize(stack_size) };
    p_task->handle_.p_thread = new std::thread([p_task]() {
        using namespace std::chrono;
        auto last_exec { system_clock::now() };
        while (p_task->state_) {
            std::this_thread::sleep_until(last_exec + milliseconds(p_task->period_));
            last_exec = system_clock::now();
            if (p_task->state_ == 1) {
                p_task->func_();
            }
        }
    });
    configASSERT(p_task->handle_.p_thread);
    free_rtos_std::gthr_freertos::set_next_stacksize(last);

    free_rtos_std::gthr_freertos::set_priority(p_task->handle_.p_thread, priority);
    free_rtos_std::gthr_freertos::set_name(p_task->handle_.p_thread, p_task->name_.c_str());

    return p_task->id_;
}

uint16_t Scheduler::task_register(const std::string& name) {
    auto task { ::xTaskGetHandle(name.c_str()) };
    return task_register(task);
}

uint16_t Scheduler::task_register(void* task) {
    if (!task) {
        return 0;
    }

    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = new Task(*this, next_id_++, ::pcTaskGetName(task), 0, ::uxTaskPriorityGet(task), nullptr);
        p_task->std_thread_ = false;
        p_task->handle_.p_freertos_handle = task;

        if (p_task->handle_.p_freertos_handle) {
            tasks_[p_task->id_] = p_task;
            return p_task->id_;
        } else {
            --next_id_;
        }
    }

    delete p_task;
    return 0;
}

bool Scheduler::task_remove(const uint16_t task_id) {
    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = tasks_.at(task_id);
        tasks_.erase(task_id);
    }
    delete p_task;

    return true;
}

uint16_t Scheduler::task_get(const std::string& name) const {
    // FIXME: lock?
    for (const auto& t : tasks_) {
        if (t.second->name_ == name) {
            return t.first;
        }
    }

    return 0xffff;
}

Task* Scheduler::task_get(const uint16_t id) const {
    // FIXME: lock?
    return tasks_.at(id);
}

bool Scheduler::task_suspend(const uint16_t id) {
    // FIXME: lock?
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 2;
    free_rtos_std::gthr_freertos::suspend(task.handle_.p_thread);

    return true;
}

bool Scheduler::task_resume(const uint16_t id) {
    // FIXME: lock?
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 1;
    free_rtos_std::gthr_freertos::resume(task.handle_.p_thread);

    return true;
}

bool Scheduler::task_join(const uint16_t id) {
    // FIXME: lock?
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.handle_.p_thread->join();

    return true;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (const auto& p_task : tasks_) {
        p_task.second->print(comm);
    }
}

void Scheduler::print_ram_usage(CommInterface& comm) const {
    const auto info { freertos::ram_usage() };

    comm.debug_printf<true>(
        PP_ARGS("free RAM: {} KB, used heap: {} KB, system free: {} KB", std::get<0>(info) / 1024UL, std::get<1>(info) / 1024UL, std::get<2>(info) / 1024UL));
}

std::unique_ptr<std::vector<std::pair<void*, float>>> Scheduler::get_runtime_stats() const {
    static std::map<void*, uint32_t> last_runtimes;
    static uint64_t last_total_runtime { 0 };

    size_t num_tasks { ::uxTaskGetNumberOfTasks() };
    std::vector<TaskStatus_t> task_data;
    task_data.resize(num_tasks);
    uint32_t total_runtime;
    num_tasks = ::uxTaskGetSystemState(task_data.data(), num_tasks, &total_runtime);

    auto current_runtimes { std::make_unique<std::vector<std::pair<void*, float>>>() };

    for (size_t i { 0 }; i < num_tasks; ++i) {
        current_runtimes->push_back(std::make_pair(
            task_data[i].xHandle, (task_data[i].ulRunTimeCounter - last_runtimes[task_data[i].xHandle]) * 100.f / (total_runtime - last_total_runtime)));
        last_runtimes[task_data[i].xHandle] = task_data[i].ulRunTimeCounter;
    }
    last_total_runtime = total_runtime;


    auto cmp = [](std::pair<void*, float> const& a, std::pair<void*, float> const& b) { return a.second >= b.second; };
    std::sort(current_runtimes->begin(), current_runtimes->end(), cmp);

    return current_runtimes;
}

} // namespace ctbot

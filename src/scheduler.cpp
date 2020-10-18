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
#include "arduino_freertos.h"


namespace ctbot {

decltype(Scheduler::p_main_task_) Scheduler::p_main_task_ {};

Scheduler::Scheduler() : next_id_ {} {
    Task* p_idle_task { new Task {
        *this, next_id_++, configIDLE_TASK_NAME, false, 0, static_cast<uint8_t>(::uxTaskPriorityGet(::xTaskGetIdleTaskHandle())), nullptr } };
    p_idle_task->std_thread_ = false;
    p_idle_task->handle_.p_freertos_handle = ::xTaskGetIdleTaskHandle();

    ::vTaskSuspendAll();
    tasks_[0] = p_idle_task;
    ::xTaskResumeAll();
}

Scheduler::~Scheduler() {
    // ::serial_puts("Scheduler::~Scheduler() entered.");
    enter_critical_section();
    for (auto& t : tasks_) {
        if (t.second->name_ != PSTR("main")) {
            // arduino::Serial.print("Scheduler::~Scheduler(): task 0x");
            // arduino::Serial.print(t.second->id_, 16);
            // arduino::Serial.print(" @ 0x");
            // arduino::Serial.print(reinterpret_cast<uintptr_t>(t.second), 16);
            // arduino::Serial.print(" will be blocked...");
            t.second->state_ = 0;
            // arduino::Serial.println(" done.");
        } else {
#ifdef __arm__
            p_main_task_ = t.second->handle_.p_thread->native_handle().get_native_handle();
#endif
        }
    }
    exit_critical_section();
    // ::serial_puts("Scheduler::~Scheduler(): all tasks blocked.");

    for (auto& t : tasks_) {
        if (t.second->name_ != PSTR("main")) {
            // arduino::Serial.print("Scheduler::~Scheduler(): task \"");
            // arduino::Serial.print(t.second->name_.c_str());
            // arduino::Serial.println("\" will be deleted...");
            // arduino::Serial.flush();
            delete t.second;
            freertos::delay_ms(100);
            // arduino::Serial.println("Scheduler::~Scheduler(): task delete done.");
            // arduino::Serial.flush();
        }
    }
    // ::serial_puts("Scheduler::~Scheduler(): all tasks deleted.");
}

void Scheduler::stop() {
    ::vTaskSuspend(p_main_task_);
    ::vTaskDelete(p_main_task_);

    // size_t num_tasks { ::uxTaskGetNumberOfTasks() };
    // std::vector<TaskStatus_t> task_data;
    // task_data.resize(num_tasks);
    // num_tasks = ::uxTaskGetSystemState(task_data.data(), num_tasks, nullptr);
    // arduino::Serial.print("number of tasks: ");
    // arduino::Serial.println(num_tasks, 10);
    // for (uint8_t i {}; i < num_tasks; ++i) {
    //     arduino::Serial.println(task_data[i].pcTaskName);
    // }

    freertos::print_ram_usage();
    // arduino::Serial.print("free stack: ");
    // arduino::Serial.println(::uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t), 10);
    ::vTaskEndScheduler();
}

size_t Scheduler::get_free_stack() const {
    return ::uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
}

size_t Scheduler::get_free_stack(const uint16_t id) const {
    auto p_task { task_get(id) };
    return ::uxTaskGetStackHighWaterMark(
               p_task->std_thread_ ? free_rtos_std::gthr_freertos::get_freertos_handle(p_task->handle_.p_thread) : p_task->handle_.p_freertos_handle)
        * sizeof(StackType_t);
}

uint16_t Scheduler::task_add(const std::string_view& name, const uint16_t period, const uint8_t priority, const uint32_t stack_size, Task::func_t&& func) {
    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = new Task { *this, next_id_++, name, false, period, priority, std::move(func) };
        tasks_[p_task->id_] = p_task;
    }

    p_task->std_thread_ = true;
    const auto last { free_rtos_std::gthr_freertos::set_next_stacksize(stack_size) };
    p_task->handle_.p_thread = new std::thread { [p_task]() {
        using namespace std::chrono;
        auto last_exec { system_clock::now() };
        while (p_task->state_) {
            std::this_thread::sleep_until(last_exec + milliseconds(p_task->period_));
            last_exec = system_clock::now();
            if (p_task->state_ == 1) {
                p_task->func_();
            }
        }
        p_task->state_ = 10; // FIXME: necessary?
    } };
    configASSERT(p_task->handle_.p_thread);
    free_rtos_std::gthr_freertos::set_next_stacksize(last);

    free_rtos_std::gthr_freertos::set_priority(p_task->handle_.p_thread, priority);
    free_rtos_std::gthr_freertos::set_name(p_task->handle_.p_thread, p_task->name_.c_str());

    return p_task->id_;
}

uint16_t Scheduler::task_register(const std::string_view& name, const bool external) {
    auto task { ::xTaskGetHandle(name.data()) };
    return task_register(task, external);
}

uint16_t Scheduler::task_register(TaskHandle_t task, const bool external) {
    if (!task) {
        return 0;
    }

    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = new Task { *this, next_id_++, ::pcTaskGetName(task), external, 0, static_cast<uint8_t>(::uxTaskPriorityGet(task)), nullptr };
        p_task->std_thread_ = false; // FIXME: could also be a std::thread
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
    // ::serial_puts("Scheduler::task_remove():");

    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        if (tasks_.count(task_id)) {
            // ::serial_puts("Scheduler::task_remove(): task_id found.");
            p_task = tasks_[task_id];
            tasks_.erase(task_id);
        } else {
            return false;
        }
    }
    delete p_task;

    // ::serial_puts("Scheduler::task_remove() done.");
    return true;
}

uint16_t Scheduler::task_get(const std::string_view& name) const {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    for (const auto& t : tasks_) {
        if (t.second->name_ == name) {
            return t.first;
        }
    }

    return 0xffff;
}

Task* Scheduler::task_get(const uint16_t id) const {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return nullptr;
    }

    return tasks_.at(id);
}

bool Scheduler::task_suspend(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 2;
    free_rtos_std::gthr_freertos::suspend(task.handle_.p_thread);

    return true;
}

bool Scheduler::task_resume(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 1;
    free_rtos_std::gthr_freertos::resume(task.handle_.p_thread);

    return true;
}

bool Scheduler::task_join(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    tasks_[id]->handle_.p_thread->join();
    return true;
}

bool Scheduler::task_set_finished(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    tasks_[id]->state_ = 0;
    return true;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (const auto& p_task : tasks_) {
        p_task.second->print(comm);
    }
}

void Scheduler::print_ram_usage(CommInterface& comm) const {
    const auto info1 { freertos::ram1_usage() };
    const auto info2 { freertos::ram2_usage() };

    comm.debug_printf<true>(PP_ARGS("RAM1 size: \x1b[1m{} KB\x1b[0m\x1b[37;40m, free RAM1: \x1b[1m\x1b[32;40m{} KB\x1b[0m\x1b[37;40m, ",
        std::get<5>(info1) / 1024UL, std::get<0>(info1) / 1024UL));
    comm.debug_printf<true>(PP_ARGS("data used: \x1b[1m\x1b[31;40m{} KB\x1b[0m\x1b[37;40m, ", std::get<1>(info1) / 1024UL));
    comm.debug_printf<true>(PP_ARGS(
        "bss used: \x1b[1m{} KB\x1b[0m\x1b[37;40m, heap used: \x1b[1m{} KB\x1b[0m\x1b[37;40m", std::get<2>(info1) / 1024UL, std::get<3>(info1) / 1024UL));
    comm.debug_printf<true>(PP_ARGS(", system free: \x1b[1m{} KB\x1b[0m\x1b[37;40m\r\n", std::get<4>(info1) / 1024UL));

    comm.debug_printf<true>(PP_ARGS("RAM2 size: \x1b[1m{} KB\x1b[0m\x1b[37;40m, free RAM2: \x1b[1m\x1b[32;40m{} KB\x1b[0m\x1b[37;40m, ",
        std::get<1>(info2) / 1024UL, std::get<0>(info2) / 1024UL));
    comm.debug_printf<true>(PP_ARGS("used RAM2: \x1b[1m\x1b[31;40m{} KB\x1b[0m\x1b[37;40m\r\n", (std::get<1>(info2) - std::get<0>(info2)) / 1024UL));
}

std::unique_ptr<std::vector<std::pair<TaskHandle_t, float>>> Scheduler::get_runtime_stats() const {
    static std::map<TaskHandle_t, uint32_t> last_runtimes;
    static uint64_t last_total_runtime { 0 };

    size_t num_tasks { ::uxTaskGetNumberOfTasks() };
    std::vector<TaskStatus_t> task_data;
    task_data.resize(num_tasks);
    uint32_t total_runtime;
    num_tasks = ::uxTaskGetSystemState(task_data.data(), num_tasks, &total_runtime);

    auto current_runtimes { std::make_unique<std::vector<std::pair<TaskHandle_t, float>>>() };

    for (size_t i { 0 }; i < num_tasks; ++i) {
        current_runtimes->push_back(std::make_pair(
            task_data[i].xHandle, (task_data[i].ulRunTimeCounter - last_runtimes[task_data[i].xHandle]) * 100.f / (total_runtime - last_total_runtime)));
        last_runtimes[task_data[i].xHandle] = task_data[i].ulRunTimeCounter;
    }
    last_total_runtime = total_runtime;


    auto cmp = [](std::pair<TaskHandle_t, float> const& a, std::pair<void*, float> const& b) { return a.second >= b.second; };
    std::sort(current_runtimes->begin(), current_runtimes->end(), cmp);

    return current_runtimes;
}

} // namespace ctbot

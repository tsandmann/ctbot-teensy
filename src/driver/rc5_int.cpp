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
 * @file    rc5_int.cpp
 * @brief   Interrupt driven RC5 decoder
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "rc5_int.h"

#include "ctbot.h"
#include "scheduler.h"

#include "rc5.h"

#include <type_traits>


namespace ctbot {

std::remove_all_extents<decltype(Rc5::input_data_)>::type Rc5::input_data_[Rc5::DATA_ARRAY_SIZE];
decltype(Rc5::input_idx_) Rc5::input_idx_ {};

Rc5::Rc5(const uint8_t pin) : last_idx_ {}, rc5_addr_ {}, rc5_cmd_ {}, rc5_toggle_ {}, p_impl_ { new RC5 {} } {
    Scheduler::enter_critical_section();
    arduino::pinMode(pin, arduino::INPUT_PULLUP);

    arduino::attachInterrupt(
        pin, []() { isr<CtBotConfig::RC5_PIN, DATA_ARRAY_SIZE>(input_data_, &input_idx_); }, arduino::CHANGE);
    Scheduler::exit_critical_section();

    reset();
}

Rc5::~Rc5() {
    delete p_impl_;
}

void Rc5::reset() {
    last_idx_ = input_idx_;
    if (p_impl_) {
        p_impl_->reset();
    }
}

bool Rc5::update() {
    if (!p_impl_) {
        return false;
    }

    const uint8_t idx { input_idx_ };
    int8_t diff_rc5 { static_cast<int8_t>(idx - last_idx_) };
    if (diff_rc5 < 0) {
        diff_rc5 += DATA_ARRAY_SIZE;
    }

    bool found {};
    if (diff_rc5) {
        if (DEBUG_) {
            CtBot& ctbot { CtBot::get_instance() };
            ctbot.get_comm()->debug_print(PSTR("\r\ndiff_rc5=%d\r\n"), diff_rc5);
        }

        for (auto i { last_idx_ }; i != idx; i = (i + 1) % DATA_ARRAY_SIZE) {
            const uint32_t i_time { input_data_[i].us };
            const auto diff_time { static_cast<int32_t>(i_time) - static_cast<int32_t>(last_time_) };
            last_time_ = i_time;

            if (DEBUG_) {
                CtBot& ctbot { CtBot::get_instance() };
                ctbot.get_comm()->debug_printf<false>(PSTR("i=%u\tus=%u\r\n"), i, input_data_[i].us);
                ctbot.get_comm()->debug_printf<false>(PSTR("i_time=%u us\tdiff_time=%d us\tvalue=%u\r\n"), i_time, diff_time, input_data_[i].value);
            }

            if (p_impl_->read(rc5_toggle_, rc5_addr_, rc5_cmd_, input_data_[i].value, diff_time)) {
                found = true;

                if (DEBUG_) {
                    CtBot& ctbot { CtBot::get_instance() };
                    ctbot.get_comm()->debug_printf<false>(PSTR("addr=0x%x\tcmd=0x%x\ttoggle=%u\r\n"), rc5_addr_, rc5_cmd_, rc5_toggle_);
                }
            }
        }
        last_idx_ = idx;
    }

    return found;
}

void Rc5::set_rc5(const uint8_t addr, const uint8_t cmd) {
    rc5_addr_ = addr;
    rc5_cmd_ = cmd;
}

void Rc5::set_rc5(const uint8_t addr, const uint8_t cmd, bool toggle) {
    rc5_toggle_ = toggle;
    rc5_addr_ = addr;
    rc5_cmd_ = cmd;
}

} // namespace ctbot

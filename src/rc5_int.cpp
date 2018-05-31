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
 * @file    rc5_int.cpp
 * @brief   Interrupt driven RC5 decoder
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "rc5_int.h"
#include "scheduler.h"

#include <rc5.h>
#include <arduino_fixed.h>
#include <type_traits>
// #include <iostream>


namespace ctbot {

std::remove_all_extents<decltype(Rc5::input_data_)>::type Rc5::input_data_[Rc5::DATA_ARRAY_SIZE];
decltype(Rc5::input_idx_) Rc5::input_idx_ { 0 };

Rc5::Rc5(const uint8_t pin) :last_idx_ { 0 }, rc5_addr_ { 0 }, rc5_cmd_ { 0 }, rc5_toggle_ { false }, p_impl_ { new RC5() } {
    Scheduler::enter_critical_section();
    arduino::pinMode(pin, INPUT_PULLUP);

    // FIXME: think about this...
    arduino::attachInterrupt(
        pin, [] () {
            static bool last { false };
            const bool value { arduino::digitalReadFast(CtBotConfig::RC5_PIN) };

            if (value != last) {
                last = value;
                isr<CtBotConfig::RC5_PIN, DATA_ARRAY_SIZE>(value, input_data_, &input_idx_);
            }
        }, CHANGE
    );
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
    if (! p_impl_) {
        return false;
    }

    const uint8_t idx { input_idx_ };
    int8_t diff_rc5 { static_cast<int8_t>(idx - last_idx_) };
    if (diff_rc5 < 0) {
        diff_rc5 += DATA_ARRAY_SIZE;
    }

    bool found { false };
    if (diff_rc5) {
        // std::cout << "idx=" << static_cast<uint16_t>(idx) << "\tlast_idx_=" << static_cast<uint16_t>(last_idx_) << "\tdiff_rc5=" << static_cast<uint16_t>(diff_rc5) << "\n";

        for (auto i(last_idx_); i != idx; i = (i + 1) % DATA_ARRAY_SIZE) {
            // std::cout << "i=" << static_cast<uint16_t>(i) << "\tus=" << input_data_[i].us;

            const uint32_t i_time { input_data_[i].us };
            const auto diff_time { static_cast<int32_t>(i_time - last_time_) };
            last_time_ = i_time;
            // std::cout << "\ti_time=" << i_time << " us\tdiff_time=" << diff_time << " us\tvalue=" << input_data_[i].value << "\n";
            // std::cout << "diff_time=" << diff_time << "\tvalue=" << input_data_[i].value << "\n";

            if (p_impl_->read(rc5_toggle_, rc5_addr_, rc5_cmd_, input_data_[i].value, diff_time)) {
                found = true;
                // std::cout << "addr=" << static_cast<uint16_t>(rc5_addr_) << "\tcmd=" << static_cast<uint16_t>(rc5_cmd_) << "\ttoggle=" << rc5_toggle_ << "\n";
            }
        }
        last_idx_ = idx;
    }

    return found;
}

} /* namespace ctbot */

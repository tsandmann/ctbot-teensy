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
 * @file    shift_reg.cpp
 * @brief   Shift register 74HC595 driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "shift_reg.h"
#include "ctbot_config.h"
#include "scheduler.h"
#include "timer.h"

#include <arduino_fixed.h>


namespace ctbot {

template <uint8_t SCK_PIN, uint8_t RCK_PIN>
ShiftReg<SCK_PIN, RCK_PIN>::ShiftReg() {
    /* set pins to output mode */
    Scheduler::enter_critical_section();
    arduino::pinMode(CtBotConfig::SHIFT_SDATA_PIN, arduino::OUTPUT);
    arduino::pinMode(RCK_PIN, arduino::OUTPUT);
    if (SCK_PIN != CtBotConfig::SHIFT_SDATA_PIN && SCK_PIN != RCK_PIN) {
        arduino::pinMode(SCK_PIN, arduino::OUTPUT);
    }
    Scheduler::exit_critical_section();
}

template <uint8_t SCK_PIN, uint8_t RCK_PIN>
void ShiftReg<SCK_PIN, RCK_PIN>::out(uint8_t data, const uint8_t, const uint8_t) const {
    Scheduler::enter_critical_section();
    arduino::digitalWriteFast(RCK_PIN, false); // reset RCK

    for (uint8_t i { 0U }; i < 8; ++i) {
        arduino::digitalWriteFast(SCK_PIN, false); // reset SCK
        /* put MSB of data on SER */
        const uint8_t tmp((data >> 7) & 1);
        arduino::digitalWriteFast(CtBotConfig::SHIFT_SDATA_PIN, tmp);
        arduino::digitalWriteFast(SCK_PIN, true); // latch to storage -> rising edge on SCK
        Timer::delay_us(1);
        data <<= 1;
    }

    // ExecuteAtomic<> x;
    // x([] () {
    // ...
    // });

    __disable_irq();
    arduino::digitalWriteFast(SCK_PIN, false);
    arduino::digitalWriteFast(CtBotConfig::SHIFT_SDATA_PIN, false);
    arduino::digitalWriteFast(RCK_PIN, true); // latch to output -> rising edge on RCK
    __enable_irq();
    Scheduler::exit_critical_section();
}

/* explicit instantiation for shift regs used by c't-Bot */
template class ShiftReg<CtBotConfig::ENA_SCK_PIN, CtBotConfig::ENA_RCK_PIN>; // ENA
template class ShiftReg<CtBotConfig::LED_SCK_PIN, CtBotConfig::LED_RCK_PIN>; // LEDs

} // namespace ctbot

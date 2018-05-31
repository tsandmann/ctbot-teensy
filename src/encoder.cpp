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
 * @file    encoder.cpp
 * @brief   c't-Bot wheel encoder driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "encoder.h"
#include "digital_sensors.h"
#include "timer.h"
#include "scheduler.h"

#include <arduino_fixed.h>
// #include <iostream>


namespace ctbot {

Encoder::Encoder(const uint32_t* p_data, const volatile uint8_t* p_idx, const uint8_t pin) :
        edges_(0), last_idx_(0), speed_(0.f), speed_avg_(0.f), direction_(true), p_enc_data_(p_data), p_enc_idx_(p_idx), last_update_(0), count_(0) {
    Scheduler::enter_critical_section();
    arduino::pinMode(pin, INPUT);

    // FIXME: think about this...
    if (pin == CtBotConfig::ENC_L_PIN) {
        arduino::attachInterrupt(
            pin, [] () {
                static bool last { false };
                const bool value { arduino::digitalReadFast(CtBotConfig::ENC_L_PIN) };

                if (value != last) {
                    last = value;
                    isr<CtBotConfig::ENC_L_PIN, DATA_ARRAY_SIZE>(DigitalSensors::enc_data_l_, &DigitalSensors::enc_l_idx_);
                }
            }, CHANGE
        );
    } else if (pin == CtBotConfig::ENC_R_PIN) {
        arduino::attachInterrupt(
            pin, [] () {
                static bool last { false };
                const bool value { arduino::digitalReadFast(CtBotConfig::ENC_R_PIN) };

                if (value != last) {
                    last = value;
                    isr<CtBotConfig::ENC_R_PIN, DATA_ARRAY_SIZE>(DigitalSensors::enc_data_r_, &DigitalSensors::enc_r_idx_);
                }
            }, CHANGE
        );
    }
    Scheduler::exit_critical_section();
}

void Encoder::update() {
    // FIXME: check data types
    const uint8_t idx { *p_enc_idx_ };
    int8_t diff_enc { static_cast<int8_t>(idx - last_idx_) };
    if (diff_enc < 0) {
        diff_enc += DATA_ARRAY_SIZE;
    }
    // std::cout << static_cast<uint16_t>(idx) << "\t" << static_cast<uint16_t>(diff_enc) << "\n";

    const auto now_us { Timer::get_us() };
    const auto dt { static_cast<int32_t>(now_us - last_update_) };

    if (diff_enc) {
        if (! direction_) {
            diff_enc = -diff_enc;
        }
        edges_ += diff_enc;
        last_idx_ = idx;
        count_ += diff_enc;

        // std::cout << static_cast<int16_t>(count_) << "\n";

        const uint32_t current_time { p_enc_data_[idx] };
        const int32_t diff { static_cast<int32_t>(current_time - last_update_) };

        if (diff == 0) {
            return;
        }

        speed_ = (WHEEL_PERIMETER / ENCODER_MARKS * 1000000.f) * count_ / diff;
        speed_avg_ = speed_avg_ * (1.f - AVG_FILTER_PARAM) + speed_ * AVG_FILTER_PARAM;

        if (speed_ != 0.f) {
            // std::cout << speed_ << "\t" << speed_avg_ << "\t" << /* current_time << "\t" << last_update_ << "\t" << */ static_cast<uint16_t>(count_) << "\n";
            // for (auto i(0U); i < 8; ++i) {
            //     std::cout << static_cast<uint16_t>(p_enc_data_[i]) << "\t";
            // }
            // std::cout << static_cast<uint16_t>(idx) << "\n";
        }
        if (count_) {
            last_update_ = current_time;
        }
        count_ = 0;
    } else if (dt > 600000L) {
        // std::cout << now_us << "\t" << last_update_ << "\t" << dt << "\n";
        speed_ = speed_avg_ = 0.f;
        count_ = 0;
        last_update_ = now_us;
    }
}

} /* namespace ctbot */

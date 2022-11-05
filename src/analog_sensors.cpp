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
 * @file    analog_sensors.cpp
 * @brief   Analog sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "analog_sensors.h"
#include "scheduler.h"
#include "ctbot.h"

#include "arduino_freertos.h"


namespace ctbot {

AnalogSensors::AnalogSensors() : last_adc_res_ {}, line_ { 0, 0 }, border_ { 0, 0 }, bat_voltage_ {}, current_5v_ {}, current_servo_ {} {
    Scheduler::enter_critical_section();
    arduino::pinMode(CtBotConfig::LINE_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::LINE_R_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BORDER_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BORDER_R_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BAT_VOLTAGE_PIN, arduino::INPUT);
    if constexpr (CtBotConfig::MAINBOARD_REVISION == 9'002) {
        arduino::pinMode(CtBotConfig::CURRENT_5V_PIN, arduino::INPUT);
        arduino::pinMode(CtBotConfig::CURRENT_SERVO_PIN, arduino::INPUT);
    }
    arduino::analogReference(0);
    Scheduler::exit_critical_section();
}

void AnalogSensors::update() {
    uint16_t volt_raw, current_5v_raw, current_servo_raw;
    {
        std::lock_guard<std::mutex> lock { adc_mutex_ };

        line_[0] = analog_read(CtBotConfig::LINE_L_PIN, 10);
        line_[1] = analog_read(CtBotConfig::LINE_R_PIN, 10);

        border_[0] = analog_read(CtBotConfig::BORDER_L_PIN, 10);
        border_[1] = analog_read(CtBotConfig::BORDER_R_PIN, 10);

        volt_raw = analog_read(CtBotConfig::BAT_VOLTAGE_PIN, BAT_ADC_RES, 16);
        current_5v_raw = analog_read(CtBotConfig::CURRENT_5V_PIN, CURRENT_ADC_RES_, 16);

        int32_t adc_servo { analog_read(CtBotConfig::CURRENT_SERVO_PIN, CURRENT_ADC_RES_, 16) - CURRENT_SERVO_ADC_OFFSET_ };
        if (CURRENT_SERVO_ADC_OFFSET_ && adc_servo < 0) {
            adc_servo = 0;
        }
        current_servo_raw = static_cast<uint16_t>(adc_servo);
    }

    bat_voltage_ = volt_raw * (3.33f * (static_cast<float>(BAT_VOLTAGE_R2 + BAT_VOLTAGE_R1) / BAT_VOLTAGE_R2) / static_cast<float>((1 << BAT_ADC_RES) - 1));

    current_5v_ = (CURRENT_MEASUREMENT_ALPHA_ * current_5v_raw * CURRENT_5V_CONVERSION_FACTOR_) + (1.f - CURRENT_MEASUREMENT_ALPHA_) * current_5v_;
    current_servo_ = (CURRENT_MEASUREMENT_ALPHA_ * current_servo_raw * CURRENT_SERVO_CONVERSION_FACTOR_) + (1.f - CURRENT_MEASUREMENT_ALPHA_) * current_servo_;

    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf(PSTR("current_5v_=%.2f mA \tcurrent_servo_=%.2f mA\r\n"), current_5v_, current_servo_);
    }
}

uint16_t AnalogSensors::analog_read(const uint8_t pin, const uint8_t resolution, const uint8_t avg_num) {
    if (pin >= 128) {
        return 0;
    }
    if (last_adc_res_ != resolution && resolution >= 8 && resolution <= 16) {
        last_adc_res_ = resolution;
        arduino::analogReadResolution(resolution);
    }
    arduino::analogReadAveraging(avg_num);
    const uint16_t ret { static_cast<uint16_t>(arduino::analogRead(pin)) };
    return ret;
}

} // namespace ctbot

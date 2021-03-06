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
 * @file    speed_control.h
 * @brief   Motor speed controller
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "speed_control.h"
#include "ctbot.h"
#include "scheduler.h"
#include "timer.h"

#include "pid_v1.h"


namespace ctbot {

std::list<SpeedControl*> SpeedControl::controller_list_;

SpeedControl::SpeedControl(Encoder& wheel_enc, Motor& motor)
    : direction_ { true }, setpoint_ {}, input_ {}, output_ {}, kp_ { 40.f }, ki_ { 30.f }, kd_ { 0.f },
      p_pid_controller_ { new Pid { input_, output_, setpoint_, kp_, ki_, kd_, true } }, wheel_encoder_ { wheel_enc }, motor_ { motor } {
    if (!p_pid_controller_) {
        return;
    }

    p_pid_controller_->set_sample_time(20U);
    p_pid_controller_->set_output_limits(0, CtBotConfig::MOT_PWM_MAX);
    p_pid_controller_->set_mode(Pid::Modes::AUTOMATIC);

    controller_list_.push_back(this);

    if (controller_list_.size() == 1) {
        CtBot::get_instance().get_scheduler()->task_add(PSTR("sctrl"), TASK_PERIOD_MS, 8, 768UL, &controller);
    }
}

SpeedControl::~SpeedControl() {
    controller_list_.remove(this);
    delete p_pid_controller_;
    if (!controller_list_.size()) {
        CtBot::get_instance().get_scheduler()->task_remove(CtBot::get_instance().get_scheduler()->task_get(PSTR("sctrl")));
    }
}

void SpeedControl::run() {
    if (!p_pid_controller_) {
        return;
    }

    input_ = std::fabs(get_enc_speed());
    p_pid_controller_->compute(Timer::get_ms());

    int pwm;
    if (setpoint_ == 0.f) {
        pwm = 0;
    } else {
        pwm = static_cast<int>(output_);
    }

    if (motor_.get() != direction_ ? pwm : -pwm) {
        motor_.set(direction_ ? pwm : -pwm);
    }
}

void SpeedControl::set_parameters(const float kp, const float ki, const float kd) {
    if (!p_pid_controller_) {
        return;
    }

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    p_pid_controller_->set_tunings(kp_, ki_, kd_);
}

void SpeedControl::controller() {
    for (auto p_ctrl : controller_list_) {
        p_ctrl->run();
    }
}

} // namespace ctbot

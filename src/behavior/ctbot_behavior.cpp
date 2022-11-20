/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
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
 * @file    ctbot_behavior.cpp
 * @brief   Behavior model abstraction layer
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "ctbot_behavior.h"

#include "actuator.h"
#include "cmd_parser.h"
#include "ctbot_cli.h"
#include "ctbot_config.h"
#include "ctbot_task.h"
#include "logger.h"
#include "pose.h"
#include "resource_container.h"
#include "scheduler.h"
#include "sensors.h"
#include "speed.h"
#include "speed_control.h"

#include "behavior_drive.h"
#include "behavior_follow_line.h"
#include "behavior_square.h"
#include "behavior_test.h"
#include "behavior_turn.h"
#include "behavior/legacy/behavior_legacy.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"

#include <cmath>


namespace ctbot {
PROGMEM const char CtBotBehavior::usage_text_beh_[] { "\r\n"
                                                      "beh (b)\r\n"
                                                      "\tstart BEHAVIOR [PARAMS]      start a behavior\r\n"
                                                      "\tstop                         stop a currently running behavior\r\n"
                                                      "\tlist                         show a list of all registered behaviors\r\n"
                                                      "\tenable                       enable the behavior subsystem\r\n"
                                                      "\tdisable                      disable the behavior subsystem\r\n"
                                                      "\tpose                         print current pose of bot\r\n"
                                                      "\r\n" };


CtBotBehavior::CtBotBehavior() : p_data_ {}, p_actuators_ {}, enc_last_l_ {}, enc_last_r_ {}, beh_enabled_ {} {}

CtBotBehavior::~CtBotBehavior() = default;

void CtBotBehavior::setup(const bool set_ready) {
    if (DEBUG_LEVEL_ >= 4) {
        ::serialport_puts(PSTR("CtBotBehavior::setup()...\r\n"));
    }
    CtBot::setup(false);

    p_cli_->add_helptext(usage_text_beh_);

    p_parser_->register_cmd(PSTR("beh"), "b", [this](const std::string_view& args) {
        if (args.find(PSTR("start")) == 0) {
            if (p_beh_) {
                return false;
            }

            const auto args2 { CmdParser::trim_to_first_arg(args) };
            const auto beh_name { args2.substr(0, args2.find(' ')) };

            if (DEBUG_LEVEL_ >= 4) {
                log_begin();
                get_logger()->log<true>(PSTR("args=\"%.*s\"\r\n"), args.size(), args.data());
                log_begin();
                get_logger()->log<true>(PSTR("args2=\"%.*s\"\r\n"), args2.size(), args2.data());
                log_begin();
                get_logger()->log<true>(PSTR("beh_name=\"%.*s\"\r\n"), beh_name.size(), beh_name.data());
            }

            const auto beh_it { behavior_list_.find(beh_name) };
            if (beh_it != behavior_list_.end()) {
                const auto beh { beh_it->second };
                const auto params { std::get<0>(beh_it->second) };
                if (DEBUG_LEVEL_ >= 3) {
                    log_begin();
                    get_logger()->log<true>(PSTR("creating behavior \"%.*s\" with %u parameter(s)...\r\n"), beh_name.size(), beh_name.data(), params);
                }

                switch (params) {
                    case 0:
                        p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>()>>(std::get<1>(beh))();
                        if (DEBUG_LEVEL_ >= 4) {
                            log_begin();
                            get_logger()->log(PSTR(" done.\r\n"), true);
                        }
                        break;

                    case 1: {
                        int32_t v;
                        if (auto [_, ec] = CmdParser::split_args(args2, v); ec == std::errc {}) {
                            p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t)>>(std::get<1>(beh))(v);
                            if (DEBUG_LEVEL_ >= 4) {
                                log_begin();
                                get_logger()->log(PSTR(" done.\r\n"), true);
                            }
                        } else {
                            return false;
                        }
                        break;
                    }

                    case 2: {
                        int32_t v1, v2;
                        if (auto [_, ec] = CmdParser::split_args(args2, v1, v2); ec == std::errc {}) {
                            p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t)>>(std::get<1>(beh))(v1, v2);
                            if (DEBUG_LEVEL_ >= 4) {
                                log_begin();
                                get_logger()->log(PSTR(" done.\r\n"), true);
                            }
                        } else {
                            return false;
                        }
                        break;
                    }

                    case 3: {
                        int32_t v1, v2, v3;
                        if (auto [_, ec] = CmdParser::split_args(args2, v1, v2, v3); ec == std::errc {}) {
                            p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t, const int32_t)>>(std::get<1>(beh))(
                                v1, v2, v3);
                            if (DEBUG_LEVEL_ >= 4) {
                                log_begin();
                                get_logger()->log(PSTR(" done.\r\n"), true);
                            }
                        } else {
                            return false;
                        }
                        break;
                    }

                    case 4: {
                        int32_t v1, v2, v3, v4;
                        if (auto [_, ec] = CmdParser::split_args(args2, v1, v2, v3, v4); ec == std::errc {}) {
                            p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t, const int32_t, const int32_t)>>(
                                std::get<1>(beh))(v1, v2, v3, v4);
                            if (DEBUG_LEVEL_ >= 4) {
                                log_begin();
                                get_logger()->log(PSTR(" done.\r\n"), true);
                            }
                        } else {
                            return false;
                        }
                        break;
                    }

                    default: return false;
                }
                return p_beh_ ? true : false;
            }
        } else if (args.find(PSTR("stop")) == 0) {
            if (p_beh_) {
                p_beh_.reset();
                return true;
            }
        } else if (args.find(PSTR("list")) == 0) {
            for (const auto& b : behavior_list_) {
                get_comm()->debug_printf<true>(PP_ARGS("{s} [{}]\r\n", b.first.c_str(), std::get<0>(b.second)));
            }
            return true;
        } else if (args.find(PSTR("enable")) == 0) {
            beh_enabled_ = true;
            return true;
        } else if (args.find(PSTR("disable")) == 0) {
            beh_enabled_ = false;
            return true;
        } else if (args.find(PSTR("pose")) == 0) {
            get_data()->get_res_ptr<Pose>(PSTR("model.pose_enc"))->print(*get_comm());
            get_comm()->debug_print(PSTR("\r\n"), false);
            return true;
        }
        return false;
    });

    p_data_ = std::make_unique<ResourceContainer>();
    configASSERT(p_data_);
    p_actuators_ = std::make_unique<ResourceContainer>();
    configASSERT(p_actuators_);

    ResourceContainer* const p_model { p_data_->create_resource<ResourceContainer>(PSTR("model."), true) };
    Resource<Pose>* const p_pose_enc { p_model->create_resource<Pose>(PSTR("pose_enc"), true) };
    p_pose_enc->register_listener([p_model](const Resource<Pose>::basetype&) {
        p_model->set_update_state((PSTR("pose_enc")));
        return;
    });

    Resource<Speed>* const p_speed_enc { p_model->create_resource<Speed>(PSTR("speed_enc"), true) };
    p_speed_enc->register_listener([p_model](const Resource<Speed>::basetype&) {
        p_model->set_update_state(PSTR("speed_enc"));
        return;
    });

    p_model->register_listener([p_model, this](const ResourceContainer&) {
        p_model->reset_update_states();
        if (DEBUG_LEVEL_ > 4) {
            log_begin();
            get_logger()->log<true>(PP_ARGS("model notify at {} ms.\r\n", Timer::get_ms()));
        }
        model_cond_.notify_all();
    });

    auto p_governors = p_actuators_->create_resource<ActuatorContainer<AMotor>>(PSTR("speed."), true);
    configASSERT(p_governors);
    auto p_governor_l = p_governors->create_actuator(PSTR("left"), true);
    auto p_governor_r = p_governors->create_actuator(PSTR("right"), true);
    configASSERT(p_governor_l && p_governor_r);
    p_governor_l->register_listener([p_governors](const AMotor::basetype&) { p_governors->set_update_state(PSTR("left")); });
    p_governor_r->register_listener([p_governors](const AMotor::basetype&) { p_governors->set_update_state(PSTR("right")); });

    p_governors->register_listener([p_governors, this](const ResourceContainer& governors) {
        if (!beh_enabled_) {
            return;
        }
        if (DEBUG_LEVEL_ > 4) {
            log_begin();
            get_logger()->log<true>(PP_ARGS("all governors set at {} ms.\r\n", Timer::get_ms()));
        }
        AMotor* p_left;
        if (governors.get_resource(PSTR("left"), p_left)) {
            const int16_t left { p_left->read() };
            p_speedcontrols_[0]->set_speed(static_cast<float>(left));
            if (DEBUG_LEVEL_ > 4) {
                log_begin();
                get_logger()->log<true>(PP_ARGS("speed left set to {}\r\n", left));
            }
        }
        AMotor* p_right;
        if (governors.get_resource(PSTR("right"), p_right)) {
            const int16_t right { p_right->read() };
            p_speedcontrols_[1]->set_speed(static_cast<float>(right));
            if (DEBUG_LEVEL_ > 4) {
                log_begin();
                get_logger()->log<true>(PP_ARGS("speed right set to {}\r\n", right));
            }
        }

        p_governors->reset_update_states();
    });

    if (CtBotConfig::BEHAVIOR_LEGACY_SUPPORT_AVAILABLE) {
        BehaviorLegacy::init();
    }

    p_actuators_->get_resource(PSTR("speed."), p_governors_);
    configASSERT(p_governors_);

    ready_ = set_ready;
    beh_enabled_ = true;

    if (DEBUG_LEVEL_ >= 4) {
        ::serialport_puts(PSTR("CtBotBehavior::setup() done.\r\n"));
    }
}

void CtBotBehavior::add_behavior_helper(const std::string_view& name, std::tuple<uint8_t, std::any>&& beh) {
    behavior_list_[std::string { name }] = beh;
}

void CtBotBehavior::register_behavior(const std::string_view& name, std::function<Behavior::BasePtr()> initializer) {
    add_behavior_helper(name, std::make_tuple(0, initializer));
}

void CtBotBehavior::register_behavior(const std::string_view& name, std::function<Behavior::BasePtr(const int32_t)> initializer) {
    add_behavior_helper(name, std::make_tuple(1, initializer));
}

void CtBotBehavior::register_behavior(const std::string_view& name, std::function<Behavior::BasePtr(const int32_t, const int32_t)> initializer) {
    add_behavior_helper(name, std::make_tuple(2, initializer));
}

void CtBotBehavior::register_behavior(const std::string_view& name, std::function<Behavior::BasePtr(const int32_t, const int32_t, const int32_t)> initializer) {
    add_behavior_helper(name, std::make_tuple(3, initializer));
}

void CtBotBehavior::register_behavior(
    const std::string_view& name, std::function<Behavior::BasePtr(const int32_t, const int32_t, const int32_t, const int32_t)> initializer) {
    add_behavior_helper(name, std::make_tuple(4, initializer));
}

void CtBotBehavior::run() {
    using namespace std::chrono_literals;
    static uint32_t last_time {};

    if (!ready_ || !beh_enabled_) {
        return;
    }

    if (DEBUG_LEVEL_ >= 4) {
        const auto now { Timer::get_ms() };
        const auto diff { now - last_time };
        last_time = now;
        if (diff > TASK_PERIOD_MS + 1) {
            log_begin();
            get_logger()->log<true>(PSTR("\nrun(): time diff=%u ms at %u ms.\r\n"), diff, now);
        }
    }

    CtBot::run();

    configASSERT(p_data_);
    auto& pose { *p_data_->get_resource<Pose>(PSTR("model.pose_enc")) };
    auto& speed { *p_data_->get_resource<Speed>(PSTR("model.speed_enc")) };
    update_enc(pose.get_ref(), speed.get_ref());

    const auto motor_requests { Behavior::get_motor_requests() };
    p_motor_sync_ = std::make_unique<std::latch>(static_cast<ptrdiff_t>(motor_requests));
    if (DEBUG_LEVEL_ >= 4 && motor_requests) {
        log_begin();
        get_logger()->log<true>(PSTR("run(): set motor barrier to %u\r\n"), motor_requests);
    }

    if (DEBUG_LEVEL_ > 4 && (speed.get_ref().get_left() || speed.get_ref().get_right())) {
        log_begin();
        pose.get_ref().print(*get_logger());
        get_logger()->log(PSTR(" "), true);
        speed.get_ref().print(*get_logger());
        get_logger()->log(PSTR("\r\n"), true);
    }

    pose.notify();
    speed.notify();


    if (DEBUG_LEVEL_ >= 4 && motor_requests) {
        log_begin();
        get_logger()->log<true>(PSTR("run(): waiting for motor barrier(%u)...\r\n"), motor_requests);
    }
    const auto start { Timer::get_ms() };
    while (!p_motor_sync_->try_wait() & (Timer::get_ms() - start < TASK_PERIOD_MS * 2)) {
        std::this_thread::sleep_for(100us);
    }
    if (motor_requests) {
        const auto barrier_done { Timer::get_ms() };
        const auto diff { barrier_done - start };
        if (diff < TASK_PERIOD_MS * 2) {
            if (DEBUG_LEVEL_ >= 4) {
                log_begin();
                get_logger()->log<true>(PSTR("run(): motor barrier done at %u ms.\r\n"), barrier_done);
            }
        } else {
            if (DEBUG_LEVEL_ >= 3) {
                log_begin();
                get_logger()->log<true>(PSTR("run(): motor barrier timeout: %u ms at %u ms.\r\n"), diff, barrier_done);
            }
        }
    }

    p_governors_->commit_values();

    if (p_beh_ && p_beh_->finished()) {
        if (DEBUG_LEVEL_ >= 4) {
            log_begin();
            get_logger()->log(PSTR("run(): deleting Behavior (p_beh_)\r\n"), true);
        }
        p_beh_.reset();
    }
}

void CtBotBehavior::wait_for_model_update(std::atomic<bool>& abort) {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lk(model_mutex_);
    while (model_cond_.wait_for(lk, 100ms) == std::cv_status::timeout) {
        if (DEBUG_LEVEL_ >= 3) {
            log_begin();
            get_logger()->log<true>(PSTR("wait_for_model_update(): TIMEOUT at %u ms, abort=%u\r\n"), Timer::get_ms(), abort.load());
        }
        if (abort) {
            return;
        }
    }
}

void CtBotBehavior::motor_update_done() {
    if (p_motor_sync_) {
        p_motor_sync_->count_down();
        if (DEBUG_LEVEL_ >= 4) {
            log_begin();
            get_logger()->log(PSTR("motor_update_done(): p_motor_sync_->count_down()\r\n"), true);
        }
    }
}

void CtBotBehavior::log_begin() const {
    get_logger()->begin(PSTR("CtBotBehavior"));
}

bool CtBotBehavior::update_enc(Pose& pose, Speed& speed) {
    using namespace std::chrono_literals;
    /* position calculation based on https://github.com/tsandmann/ct-bot/blob/master/sensor.c#L232 by Torsten Evers */

    const int32_t enc_l { p_speedcontrols_[0]->get_enc_counts() };
    const int32_t enc_r { p_speedcontrols_[1]->get_enc_counts() };

    int32_t diff_l { enc_l - enc_last_l_ };
    if (std::abs(diff_l) > 1'000) { // error check
        if (DEBUG_LEVEL_ >= 4) {
            log_begin();
            get_logger()->log<true>(PSTR("update_enc(): diff_l > 200: %d\tenc_l=%d\tenc_last_l=%d\r\n"), diff_l, enc_l, enc_last_l_);
        }
        diff_l = 0;
    }

    int32_t diff_r { enc_r - enc_last_r_ };
    if (std::abs(diff_r) > 1'000) { // error check
        if (DEBUG_LEVEL_ >= 4) {
            log_begin();
            get_logger()->log<true>(PSTR("update_enc(): diff_r > 200: %d\tenc_r=%d\tenc_last_r=%d\r\n"), diff_r, enc_r, enc_last_r_);
        }
        diff_r = 0;
    }

    if (diff_l != 0 || diff_r != 0) {
        enc_last_l_ = enc_l;
        enc_last_r_ = enc_r;

        const float d_sl { static_cast<float>(diff_l * (CtBotConfig::WHEEL_PERIMETER / static_cast<float>(CtBotConfig::ENCODER_MARKS))) };
        const float d_sr { static_cast<float>(diff_r * (CtBotConfig::WHEEL_PERIMETER / static_cast<float>(CtBotConfig::ENCODER_MARKS))) };
        float d_head { (d_sr - d_sl) / CtBotConfig::WHEEL_TO_WHEEL_DISTANCE }; // angle in radians

        /* calculate heading */
        float delta_y;
        if (d_head == 0.f) {
            /* straight ahead: delta_y = diff_l = diff_r */
            delta_y = d_sl;
        } else {
            /* calculate displacement from alpha/2 */
            delta_y = (d_sl + d_sr) * std::sin(d_head / 2.f) / d_head;
            d_head = d_head / (static_cast<float>(M_PI) / 180.f); // convert angle in degree

            float heading { pose.get_heading<float>() };
            heading += d_head;
            if (heading >= 360.f) {
                heading -= 360.f;
            } else if (heading < 0.f) {
                heading += 360.f;
            }
            pose.set_heading(heading);
        }

        /* calculate new position */
        if (std::fabs(delta_y) > 0.f) {
            float x { pose.get_x<float>() };
            float y { pose.get_y<float>() };
            x += delta_y * pose.get_heading_cos<float>();
            y += delta_y * pose.get_heading_sin<float>();
            pose.set_x(x);
            pose.set_y(y);
        }
    }

    const float speed_l { p_speedcontrols_[0]->get_enc_speed() };
    const float speed_r { p_speedcontrols_[1]->get_enc_speed() };
    speed.set_left(speed_l);
    speed.set_right(speed_r);
    speed.set_center((speed_l + speed_r) / 2.f);

    return true;
}

void CtBotBehavior::shutdown() {
    if (DEBUG_LEVEL_ >= 3) {
        log_begin();
        get_logger()->log(PSTR("shutdown()\r\n"), true);
        get_logger()->flush();
    }

    ready_ = false;

    p_beh_.reset();
    if (CtBotConfig::BEHAVIOR_LEGACY_SUPPORT_AVAILABLE) {
        delete BehaviorLegacy::get_instance();
    }

    auto& pose { *p_data_->get_resource<Pose>(PSTR("model.pose_enc")) };
    auto& speed { *p_data_->get_resource<Speed>(PSTR("model.speed_enc")) };
    pose.notify();
    speed.notify();

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(200ms);

    CtBot::shutdown();
}

} // namespace ctbot

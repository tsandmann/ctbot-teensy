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
#include "ctbot_config.h"
#include "help_texts.h"
#include "cmd_parser.h"
#include "resource_container.h"
#include "actuator.h"
#include "sensors.h"
#include "pose.h"
#include "speed.h"
#include "scheduler.h"
#include "ctbot_task.h"
#include "speed_control.h"
#include "behavior/legacy/behavior_legacy.h"
#include "behavior_test.h"
#include "behavior_turn.h"
#include "behavior_drive.h"
#include "behavior_square.h"
#include "behavior_follow_line.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"

#include <cmath>


namespace ctbot {
PROGMEM const char CtBotBehavior::usage_text_beh[] { "\r\n"
                                                     "beh (b)\r\n"
                                                     "\tstart BEHAVIOR [PARAMS]\tstart a behavior\r\n"
                                                     "\tstop\t\t\tstop a currently running behavior\r\n"
                                                     "\tlist\t\t\tshow a list of all registered behaviors\r\r"
                                                     "\r\n" };

CtBotBehavior::CtBotBehavior() : p_data_ {}, p_actuators_ {}, enc_last_l_ {}, enc_last_r_ {}, beh_enabled_ {} {}

void CtBotBehavior::setup(const bool set_ready) {
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBotBehavior::setup()...\r\n"));
    }
    CtBot::setup(false);

    p_parser_->register_cmd(PSTR("help"), 'h', [this](const std::string_view&) FLASHMEM {
        CtBotHelpTexts::print(*p_comm_);
        p_comm_->debug_print(usage_text_beh, true);
        return true;
    });

    p_parser_->register_cmd(PSTR("beh"), 'b', [this](const std::string_view& args) FLASHMEM {
        if (args.find(PSTR("start")) == 0) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            const std::string_view beh_name { args.substr(s, e - s) };

            if (p_beh_) {
                return false;
            }
            const auto beh_it { behavior_list_.find(beh_name) };
            if (beh_it != behavior_list_.end()) {
                const auto beh { beh_it->second };
                const auto params { std::get<0>(beh_it->second) };
                get_comm()->debug_printf<true>(PSTR("creating behavior \"%.*s\" with %u parameters...\r\n"), beh_name.size(), beh_name.data(), params);

                switch (params) {
                    case 0:
                        p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>()>>(std::get<1>(beh))();
                        get_comm()->debug_print(PSTR(" done.\r\n"), true);
                        break;

                    case 1: {
                        int32_t v {};
                        CmdParser::split_args(args.substr(e), v);
                        p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t)>>(std::get<1>(beh))(v);
                        get_comm()->debug_print(PSTR(" done.\r\n"), true);
                        break;
                    }

                    case 2: {
                        int32_t v1 {}, v2 {};
                        CmdParser::split_args(args.substr(e), v1, v2);
                        p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t)>>(std::get<1>(beh))(v1, v2);
                        get_comm()->debug_print(PSTR(" done.\r\n"), true);
                        break;
                    }

                    case 3: {
                        int32_t v1 {}, v2 {}, v3 {};
                        CmdParser::split_args(args.substr(e), v1, v2, v3);
                        p_beh_ =
                            std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t, const int32_t)>>(std::get<1>(beh))(v1, v2, v3);
                        get_comm()->debug_print(PSTR(" done.\r\n"), true);
                        break;
                    }

                    case 4: {
                        int32_t v1 {}, v2 {}, v3 {}, v4 {};
                        CmdParser::split_args(args.substr(e), v1, v2, v3, v4);
                        p_beh_ = std::any_cast<std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t, const int32_t, const int32_t)>>(
                            std::get<1>(beh))(v1, v2, v3, v4);
                        get_comm()->debug_print(PSTR(" done.\r\n"), true);
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
            } else {
                return false;
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
        } else {
            return false;
        }
        return true;
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
        model_cond_.notify_all();
        p_model->reset_update_states();
    });

    auto p_governors = p_actuators_->create_resource<ActuatorContainer<AMotor>>(PSTR("speed."), true);
    configASSERT(p_governors);
    auto p_governor_l = p_governors->create_actuator(PSTR("left"), true);
    auto p_governor_r = p_governors->create_actuator(PSTR("right"), true);
    configASSERT(p_governor_l && p_governor_r);
    p_governor_l->register_listener([p_governors](const AMotor::basetype&) {
        p_governors->set_update_state((PSTR("left")));
        return;
    });
    p_governor_r->register_listener([p_governors](const AMotor::basetype&) {
        p_governors->set_update_state(PSTR("right"));
        return;
    });

    p_governors->register_listener([p_governors, this](const ResourceContainer& governors) {
        if (!beh_enabled_) {
            return;
        }
        // get_comm()->debug_printf<true>(PP_ARGS("all governors set at {} ms.\r\n", Timer::get_ms()));
        AMotor* p_left;
        if (governors.get_resource(PSTR("left"), p_left)) {
            const int16_t left { p_left->read() };
            p_speedcontrols_[0]->set_speed(static_cast<float>(left));
            // get_comm()->debug_printf<true>(PP_ARGS("speed left set to {}\r\n", left));
        }
        AMotor* p_right;
        if (governors.get_resource(PSTR("right"), p_right)) {
            const int16_t right { p_right->read() };
            p_speedcontrols_[1]->set_speed(static_cast<float>(right));
            // get_comm()->debug_printf<true>(PP_ARGS("speed right set to {}\r\n", right));
        }
        p_governors->reset_update_states();
        return;
    });

    if (CtBotConfig::BEHAVIOR_LEGACY_SUPPORT_AVAILABLE) {
        BehaviorLegacy::init();
    }

    ready_ = set_ready;
    beh_enabled_ = true;

    if (DEBUG_LEVEL_ > 2) {
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
    if (!ready_ || !beh_enabled_) {
        return;
    }

    CtBot::run();

    configASSERT(p_data_);
    auto& pose { *p_data_->get_resource<Pose>(PSTR("model.pose_enc")) };
    auto& speed { *p_data_->get_resource<Speed>(PSTR("model.speed_enc")) };
    update_enc(pose.get_ref(), speed.get_ref());

    if (CtBotConfig::BEHAVIOR_LEGACY_SUPPORT_AVAILABLE) {
        BehaviorLegacy::update_global_data();
    }

    pose.notify();
    speed.notify();

    // FIXME: use a condition to indicate behavior execution fisnished?
    // get_comm()->debug_print("waiting for beh. condition.\r\n");
    // std::unique_lock<std::mutex> lk(beh_mutex_);
    // beh_cond_.wait(lk);
    // get_comm()->debug_print("got beh. condition.\r\n");

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(5ms); // FIXME: wait time?

    ActuatorContainer<AMotor>* p_governors;
    p_actuators_->get_resource(PSTR("speed."), p_governors);
    configASSERT(p_governors);

    p_governors->commit_values();

    if (p_beh_ && p_beh_->finished()) {
        get_comm()->debug_print(PSTR("deleting Behavior (p_beh_)\r\n"), true);
        p_beh_.reset();
    }
}

void CtBotBehavior::wait_for_model_update(std::atomic<bool>& abort) {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lk(model_mutex_);
    while (model_cond_.wait_for(lk, 10ms) == std::cv_status::timeout) {
        if (abort) {
            return;
        }
    }
}

bool CtBotBehavior::update_enc(Pose& pose, Speed& speed) {
    /* position calculation based on https://github.com/tsandmann/ct-bot/blob/master/sensor.c#L232 by Torsten Evers */

    const int16_t enc_l { p_sensors_->get_enc_l().get() };
    const int16_t enc_r { p_sensors_->get_enc_r().get() };

    /* check for under-/overflow */
    int32_t diff_l { enc_l - enc_last_l_ };
    if (diff_l > 255) {
        // get_comm()->debug_print("CtBotBehavior::update_enc(): diff_l > 255\r\n", true);
        diff_l -= 32'768;
    } else if (diff_l < -255) {
        // get_comm()->debug_print("CtBotBehavior::update_enc(): diff_l < -255\r\n", true);
        diff_l += 32'768;
    }
    int32_t diff_r { enc_r - enc_last_r_ };
    if (diff_r > 255) {
        // get_comm()->debug_print("CtBotBehavior::update_enc(): diff_r > 255\r\n", true);
        diff_r -= 32'768;
    } else if (diff_r < -255) {
        // get_comm()->debug_print("CtBotBehavior::update_enc(): diff_r < -255\r\n", true);
        diff_r += 32'768;
    }

    if (diff_l != 0 || diff_r != 0) {
        enc_last_l_ = enc_l;
        enc_last_r_ = enc_r;

        const float d_sl { static_cast<float>(diff_l * (178.1283f / static_cast<float>(CtBotConfig::ENCODER_MARKS))) }; // FIXME: constant for wheel perimeter
        const float d_sr { static_cast<float>(diff_r * (178.1283f / static_cast<float>(CtBotConfig::ENCODER_MARKS))) };
        float d_head { (d_sr - d_sl) / 97.2f }; // angle in radians

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

    const float speed_l { p_sensors_->get_enc_l().get_speed() };
    const float speed_r { p_sensors_->get_enc_r().get_speed() };
    speed.set_left(speed_l);
    speed.set_right(speed_r);
    speed.set_center((speed_l + speed_r) / 2.f);

    return true;
}

void CtBotBehavior::shutdown() {
    get_comm()->debug_print(PSTR("CtBotBehavior::shutdown()\r\n"), true);
    get_comm()->flush();

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

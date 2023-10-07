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
 * @file    ctbot_behavior.h
 * @brief   Behavior model abstraction layer
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "../ctbot.h"

#include "actuator.h"
#include "resource_container.h"

#include <any>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <latch>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <tuple>


class Kalman;

namespace ctbot {
class Behavior;
class Pose;
class Speed;

/**
 * @brief Main class of ct-Bot teensy framework with behaviors
 *
 * @startuml{CtBotBehavior.png}
 *  !include behavior/ctbot_behavior.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class CtBotBehavior : public CtBot {
    static constexpr uint8_t DEBUG_LEVEL_ { 3 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose, 5: noisy

protected:
    friend class CtBot; // allows protected constructor to enforce singleton pattern

    PROGMEM static const char usage_text_beh_[]; /**< C-String containing the usage / help message */

    ResourceContainer::Ptr p_data_; /**< Top level resource container for sensor models */
    ResourceContainer::Ptr p_actuators_; /**< Top level resource container for for actuators */
    std::mutex model_mutex_;
    std::unique_ptr<std::latch> p_motor_sync_;
    std::condition_variable model_cond_; /**< Pointer to condition for sensor model updates */
    int32_t enc_last_l_; /**< Last value of left wheel encoder */
    int32_t enc_last_r_; /**< Last value of right wheel encoder */
    Kalman* p_heading_filter_;
    uint32_t last_heading_update_;
    float last_heading_;
    std::map<const std::string /* name */, std::tuple<uint8_t /* # parameter */, std::any /* factory functor */>, std::less<>>
        behavior_list_; /**< List of all registered behaviors */
    std::unique_ptr<Behavior> p_beh_; /**< Pointer to currently running behavior started from command line */
    bool beh_enabled_;
    ActuatorContainer<AMotor>* p_governors_;

    /**
     * @brief Constructor of main class
     * @note Constructor is protected to enforce singleton pattern
     */
    FLASHMEM CtBotBehavior();

    /* enforce singleton */
    CtBotBehavior(const CtBotBehavior&) = delete;
    void operator=(const CtBotBehavior&) = delete;
    CtBotBehavior(CtBotBehavior&&) = delete;

    /**
     * @brief Main task implementation
     */
    virtual void run() override;

    FLASHMEM virtual void shutdown() override;

    /**
     * @brief Update pose and speed based on new wheel encoder data
     *
     * @param[out] pose Reference to a Pose instance to be updated
     * @param[out] speed Reference to a Speed instance to be updated
     */
    void update_enc(Pose& pose, Speed& speed);

    bool update_gyro(const Pose& pose);

    FLASHMEM void add_behavior_helper(const std::string_view& name, std::tuple<uint8_t, std::any>&& beh);

    FLASHMEM void log_begin() const;

public:
    /**
     * @brief Destroy the CtBotBehavior instance
     */
    FLASHMEM virtual ~CtBotBehavior() override;

    /**
     * @see CtBot::setup()
     */
    FLASHMEM virtual void setup(const bool set_ready) override;

    FLASHMEM void register_behavior(const std::string_view& name, std::function<std::unique_ptr<Behavior>()> initializer);

    FLASHMEM void register_behavior(const std::string_view& name, std::function<std::unique_ptr<Behavior>(const int32_t)> initializer);

    FLASHMEM void register_behavior(const std::string_view& name, std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t)> initializer);

    FLASHMEM void register_behavior(
        const std::string_view& name, std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t, const int32_t)> initializer);

    FLASHMEM void register_behavior(
        const std::string_view& name, std::function<std::unique_ptr<Behavior>(const int32_t, const int32_t, const int32_t, const int32_t)> initializer);

    /**
     * @brief Get the one and only instance of this class (singleton)
     * @return Reference to CtBotBehavior instance
     */
    static CtBotBehavior& get_instance() {
        return reinterpret_cast<CtBotBehavior&>(CtBot::get_instance());
    }

    /**
     * @brief Get the data resource container
     *
     * @return Pointer to data resource container
     */
    ResourceContainer* get_data() {
        return p_data_.get();
    }

    /**
     * @brief Get the actuator resource container
     *
     * @return Pointer to actuator resource container
     */
    ResourceContainer* get_actuators() {
        return p_actuators_.get();
    }

    void wait_for_model_update(std::atomic<bool>& abort);

    void motor_update_done();
};
} // namespace ctbot

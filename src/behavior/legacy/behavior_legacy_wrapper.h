/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    behavior_legacy_wrapper.h
 * @brief   Support layer for ct-Bot legacy behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "../behavior.h"
#include "behavior_legacy.h"

#include <tuple>
#include <unordered_map>


namespace ctbot {

class BehaviorLegacyWrapper : public Behavior {
    static constexpr bool DEBUG_ { true };
    static constexpr uint32_t STACK_SIZE { 4096 };

protected:
    template <typename... Args>
    static Registry REGISTRY_HELPER(const std::string_view&& name, Args&&... args) {
        if (CtBotConfig::BEHAVIOR_LEGACY_SUPPORT_AVAILABLE) {
            return Registry { name, args... };
        } else {
            return Registry { "" };
        }
    }

    static std::unordered_map<std::string /*name*/, legacy::Behaviour_t* /* pointer to behavior */> running_behaviors;

    const legacy::BehaviourFunc_t beh_func_;

    virtual void start() = 0;
    virtual void run() override;
    void cleanup();

public:
    BehaviorLegacyWrapper(const std::string& name, legacy::BehaviourFunc_t func);
    virtual ~BehaviorLegacyWrapper() override;

    static void clear_all();
};


class BehaviorDriveDistance : public BehaviorLegacyWrapper {
    static constexpr bool DEBUG_ { true };
    static Registry reg_;


public:
    BehaviorDriveDistance(const int8_t curve, const int16_t speed, const int16_t cm);

protected:
    const int8_t curve_;
    const int16_t speed_;
    const int16_t length_;

    virtual void start() override;
};


class BehaviorSimple : public BehaviorLegacyWrapper {
    static constexpr bool DEBUG_ { true };
    static Registry reg_;

public:
    BehaviorSimple();

protected:
    virtual void start() override;
};


class BehaviorGotoPos : public BehaviorLegacyWrapper {
    static constexpr bool DEBUG_ { true };
    static Registry reg1_;
    static Registry reg2_;

public:
    BehaviorGotoPos(int16_t x, int16_t y, int16_t heading, bool relative = false);
    BehaviorGotoPos(int16_t distance, int8_t direction);
    BehaviorGotoPos(int16_t distance, int8_t direction, int16_t heading);

protected:
    const int16_t x_;
    const int16_t y_;
    const int16_t head_;
    const bool rel_;
    const int16_t dist_;
    const int8_t dir_;

    virtual void start() override;
};


class BehaviorDriveSquareLegacy : public BehaviorLegacyWrapper {
    static constexpr bool DEBUG_ { true };
    static Registry reg1_;
    static Registry reg2_;

public:
    BehaviorDriveSquareLegacy();
    BehaviorDriveSquareLegacy(uint16_t length);

protected:
    const uint16_t length_;

    virtual void start() override;
};

class BehaviorAdventcal : public BehaviorLegacyWrapper {
    static constexpr bool DEBUG_ { true };
    static Registry reg_;

public:
    BehaviorAdventcal();

protected:
    virtual void start() override;
};

} /* namespace ctbot */

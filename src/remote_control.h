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
 * @file    remote_control.h
 * @brief   Remote control
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "driver/rc5_int.h"

#include <functional>
#include <map>


namespace ctbot {

/**
 * @brief Remote control implementation
 *
 * @startuml{RemoteControl.png}
 *  !include remote_control.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class RemoteControl {
protected:
    using func_t = std::function<bool(uint8_t)>;

    Rc5& rc5_;
    const uint8_t addr_;
    bool last_toggle_;
    uint8_t last_cmd_;
    std::map<uint8_t /*cmd*/, func_t /*function*/> key_mappings_;

    void change_speed(bool right, float diff) const;

public:
    /**
     * @brief Construct a new RemoteControl object
     * @param[in] rc5: Reference to RC5 decoder instance
     * @param[in] rc5_address: RC5 address of remote control for filtering
     */
    FLASHMEM RemoteControl(Rc5& rc5, const uint8_t rc5_address);

    /**
     * @brief Register a RC5 command with an action to be executed when receiving this command
     * @param[in] cmd: RC5 command to register an action for
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    FLASHMEM void register_cmd(const uint8_t cmd, func_t&& func);

    /**
     * @brief Check for new RC5 data and execute a registered handler for a received command (if any)
     */
    void update();
};

} // namespace ctbot

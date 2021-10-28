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
 * @file    behavior_test.h
 * @brief   Test behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "behavior.h"

#include <cstdint>
#include <memory>


namespace ctbot {

/**
 * @brief Test behavior
 *
 * @startuml{BehaviorTest.png}
 *  !include behavior/behavior_test.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class BehaviorTest : public Behavior {
    // FIXME: add documentation
public:
    using Ptr = std::unique_ptr<BehaviorTest>;

    FLASHMEM BehaviorTest();

protected:
    FLASHMEM virtual void run() override;

private:
    static const Registry reg_;
    uint8_t mode_;
};

} // namespace ctbot

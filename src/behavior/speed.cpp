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
 * @file    speed.cpp
 * @brief   Speed representation
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "speed.h"
#include "logger.h"

#include "pprintpp.hpp"


namespace ctbot {
void Speed::print(Logger& logger) const {
    logger.log<false>(PP_ARGS("\\{v_left={} v_right={} v_center={}}", get_left(), get_right(), get_center()));
}
} // namespace ctbot

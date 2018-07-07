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
 * @file    digital_sensors.cpp
 * @brief   Digital sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "digital_sensors.h"

#include <arduino_fixed.h>
#include <type_traits>


namespace ctbot {

std::remove_all_extents<decltype(DigitalSensors::enc_data_l_)>::type DigitalSensors::enc_data_l_[Encoder::DATA_ARRAY_SIZE],
    DigitalSensors::enc_data_r_[Encoder::DATA_ARRAY_SIZE];
decltype(DigitalSensors::enc_l_idx_) DigitalSensors::enc_l_idx_, DigitalSensors::enc_r_idx_;

DigitalSensors::DigitalSensors()
    : shutter_(false), transport_(false), enc_l_(enc_data_l_, &enc_l_idx_, CtBotConfig::ENC_L_PIN), enc_r_(enc_data_r_, &enc_r_idx_, CtBotConfig::ENC_R_PIN),
      rc5_(CtBotConfig::RC5_PIN), remote_control_(rc5_, CtBotConfig::RC5_ADDR) {
    arduino::pinMode(CtBotConfig::SHUTTER_PIN, INPUT);
    arduino::pinMode(CtBotConfig::TRANSPORT_PIN, INPUT);
}

void DigitalSensors::update() {
    shutter_ = arduino::digitalReadFast(CtBotConfig::SHUTTER_PIN);
    transport_ = arduino::digitalReadFast(CtBotConfig::TRANSPORT_PIN);

    enc_l_.update();
    enc_r_.update();
    rc5_.update();
    remote_control_.update();
}

} // namespace ctbot

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
 * @file    ena_pca9554.h
 * @brief   Abstraction layer for ct-Bot enables via PCA9554
 * @author  Timo Sandmann
 * @date    01.11.2019
 */

#pragma once

#include "ctbot_config.h"
#include "i2c_service.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Enum class for all enable signals
 *
 * @startuml{EnaI2cTypes.png}
 *  !include enai2c.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
enum class EnaI2cTypes : uint8_t {
    NONE = 0,
    DISTANCE_L = 1 << 0,
    TRANSPORT = 1 << 1,
    DISTANCE_R = 1 << 2,
    AUDIO = 1 << 3,
    EXTENSION_1 = 1 << 4,
    EXTENSION_2 = 1 << 5,
    MOTOR_RESET = 1 << 6,
    IMU_RESET = 1 << 7,
};

/**
 * @brief OR operator for EnaI2cTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs OR rhs
 */
inline constexpr EnaI2cTypes operator|(EnaI2cTypes lhs, EnaI2cTypes rhs) {
    return static_cast<EnaI2cTypes>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

/**
 * @brief AND operator for EnaI2cTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs AND rhs
 */
inline constexpr EnaI2cTypes operator&(EnaI2cTypes lhs, EnaI2cTypes rhs) {
    return static_cast<EnaI2cTypes>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

/**
 * @brief NOT operator for EnaI2cTypes
 * @param[in] rhs: Right hand side operand
 * @return NOT rhs
 */
inline constexpr EnaI2cTypes operator~(EnaI2cTypes rhs) {
    return static_cast<EnaI2cTypes>(~static_cast<uint8_t>(rhs));
}

/**
 * @brief Enable driver
 *
 * @startuml{EnaI2c.png}
 *  !include enai2c.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class EnaI2c {
    static constexpr uint8_t INPUT_PORT_REG { 0 };
    static constexpr uint8_t OUTPUT_PORT_REG { 1 };
    static constexpr uint8_t POL_INVERSION_REG { 2 };
    static constexpr uint8_t CONFIG_REG { 3 };

    I2C_Service* p_i2c_svc_;
    const uint8_t i2c_addr_;

    I2C_Service::I2C_Error write_reg8(uint8_t reg, uint8_t value) const {
        return p_i2c_svc_->write_reg(i2c_addr_, reg, value);
    }

protected:
    bool init_;
    EnaI2cTypes status_;

    /**
     * @brief Updates the PCA9554 to set the ENA state
     */
    bool update() const {
        if (init_) {
            return write_reg8(OUTPUT_PORT_REG, static_cast<uint8_t>(status_)) == I2C_Service::I2C_Error::SUCCESS;
        }

        return false;
    }

public:
    EnaI2c(I2C_Service* p_i2c_svc, uint8_t i2c_addr)
        : p_i2c_svc_ { p_i2c_svc }, i2c_addr_ { i2c_addr }, init_ {},
          status_ { CtBotConfig::MAINBOARD_REVISION == 9'000 ? EnaI2cTypes::EXTENSION_2 | EnaI2cTypes::MOTOR_RESET : EnaI2cTypes::NONE } {
        write_reg8(CONFIG_REG,
            static_cast<uint8_t>(CtBotConfig::MAINBOARD_REVISION == 9'000 ? EnaI2cTypes::NONE : EnaI2cTypes::EXTENSION_2 | EnaI2cTypes::MOTOR_RESET));
        init_ = true;
    }

    /**
     * @brief Enables one or more output pins given by the mask enable
     * @param[in] enable: Bitmask of ENA pins to be enabled
     * @note Pins not selected by the mask are uneffected
     */
    bool on(EnaI2cTypes enable) {
        status_ = status_ | enable;
        return update();
    }

    /**
     * @brief Disables one or more output pins given by the mask disable
     * @param[in] disable: Bitmask of ENA pins to be disabled
     * @note Pins not selected by the mask are uneffected
     */
    bool off(EnaI2cTypes disable) {
        status_ = status_ & (~disable);
        return update();
    }

    /**
     * @brief Enables the output pins given by the mask, disables the other ones
     * @param[in] mask: Bitmask of ENA pins to be set
     */
    bool set(EnaI2cTypes mask) {
        status_ = mask;
        return update();
    }
};

} // namespace ctbot

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
 * @file    leds_i2c.h
 * @brief   Abstraction layer for ct-Bot LED (I2C) switching
 * @author  Timo Sandmann
 * @date    01.11.2019
 */

#pragma once

#include "ctbot_config.h"
#include "i2c_service.h"

#include <array>
#include <cstdint>
#include <type_traits>


namespace ctbot {

/**
 * @brief Enum class for LEDs
 *
 * @startuml{LedTypes.png}
 *  !include leds.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
enum class LedTypes : uint8_t {
    NONE = 0,
    RIGHT = 1 << 0,
    LEFT = 1 << 1,
    RED = 1 << 2,
    ORANGE = 1 << 3,
    YELLOW = 1 << 4,
    GREEN = 1 << 5,
    BLUE = 1 << 6,
    WHITE = 1 << 7,
};

/**
 * @brief OR operator for LedTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs OR rhs
 */
inline constexpr LedTypes operator|(LedTypes lhs, LedTypes rhs) {
    return static_cast<LedTypes>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

/**
 * @brief AND operator for LedTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs AND rhs
 */
inline constexpr LedTypes operator&(LedTypes lhs, LedTypes rhs) {
    return static_cast<LedTypes>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

/**
 * @brief NOT operator for LedTypes
 * @param[in] rhs: Right hand side operand
 * @return NOT rhs
 */
inline constexpr LedTypes operator~(LedTypes rhs) {
    return static_cast<LedTypes>(~static_cast<uint8_t>(rhs));
}


/**
 * @brief Enum class for LEDs ENA on HW 0.9.0
 *
 * @startuml{LedTypesEna.png}
 *  !include leds.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
enum class LedTypesEna_9000 : uint8_t {
    NONE = 0,
    ENC_L = 1 << 0,
    ENC_R = 1 << 1,
    BORDER_L = 1 << 2,
    BORDER_R = 1 << 3,
    LINE_L = 1 << 4,
    LINE_R = 1 << 5,
    EXTENSION_1 = 1 << 6,
    EXTENSION_2 = 1 << 7,
};

/**
 * @brief OR operator for LedTypesEna_9000
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs OR rhs
 */
inline constexpr LedTypesEna_9000 operator|(LedTypesEna_9000 lhs, LedTypesEna_9000 rhs) {
    return static_cast<LedTypesEna_9000>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

/**
 * @brief AND operator for LedTypesEna_9000
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs AND rhs
 */
inline constexpr LedTypesEna_9000 operator&(LedTypesEna_9000 lhs, LedTypesEna_9000 rhs) {
    return static_cast<LedTypesEna_9000>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

/**
 * @brief NOT operator for LedTypesEna_9000
 * @param[in] rhs: Right hand side operand
 * @return NOT rhs
 */
inline constexpr LedTypesEna_9000 operator~(LedTypesEna_9000 rhs) {
    return static_cast<LedTypesEna_9000>(~static_cast<uint8_t>(rhs));
}


/**
 * @brief Enum class for LEDs ENA on HW 0.9.2
 *
 * @startuml{LedTypesEna.png}
 *  !include leds.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
enum class LedTypesEna_9002 : uint8_t {
    NONE = 0,
    BORDER_L = 1 << 0,
    BORDER_R = 1 << 1,
    EXTENSION_2 = 1 << 2,
    EXTENSION_1 = 1 << 3,
    LINE_L = 1 << 4,
    LINE_R = 1 << 5,
    TFT_BACKL = 1 << 6,
    EXTENSION_3 = 1 << 7,
};

/**
 * @brief OR operator for LedTypesEna_9002
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs OR rhs
 */
inline constexpr LedTypesEna_9002 operator|(LedTypesEna_9002 lhs, LedTypesEna_9002 rhs) {
    return static_cast<LedTypesEna_9002>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

/**
 * @brief AND operator for LedTypesEna_9002
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs AND rhs
 */
inline constexpr LedTypesEna_9002 operator&(LedTypesEna_9002 lhs, LedTypesEna_9002 rhs) {
    return static_cast<LedTypesEna_9002>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

/**
 * @brief NOT operator for LedTypesEna_9002
 * @param[in] rhs: Right hand side operand
 * @return NOT rhs
 */
inline constexpr LedTypesEna_9002 operator~(LedTypesEna_9002 rhs) {
    return static_cast<LedTypesEna_9002>(~static_cast<uint8_t>(rhs));
}

template <uint32_t HW_REV = CtBotConfig::MAINBOARD_REVISION>
using LedTypesEna = std::conditional_t<(HW_REV > 9'000), LedTypesEna_9002, LedTypesEna_9000>;


template <class T>
class LedsI2cBase {
    static constexpr uint8_t MODE1_REG { 0 }; // mode register 1
    static constexpr uint8_t MODE2_REG { 1 }; // mode register 2
    static constexpr uint8_t LEDOUT0_REG { 0xc }; // LED output state register 0
    static constexpr uint8_t LEDOUT1_REG { 0xd }; // LED output state register 1

    I2C_Service* p_i2c_svc_;
    const uint8_t i2c_addr_;
    T status_;
    bool init_;

    uint8_t write_reg8(const uint8_t reg, const uint8_t value) const {
        return p_i2c_svc_->write_reg(i2c_addr_, reg, value);
    }

protected:
    std::array<uint8_t, 8> pwm_dt_;

    FLASHMEM LedsI2cBase(I2C_Service* p_i2c_svc, const uint8_t i2c_addr)
        : p_i2c_svc_ { p_i2c_svc }, i2c_addr_ { i2c_addr }, status_ { static_cast<T>(LedTypesEna<>::NONE) }, init_ {} {
        // FIXME: read initial status from device
        if (write_reg8(MODE1_REG, 0)) {
            return;
        }
        if (write_reg8(MODE2_REG, 5)) { // OUTDRV | OUTNE1
            return;
        }
        if (write_reg8(LEDOUT0_REG, 0b10101010)) { // LED driver x individual brightness can be controlled through its PWMx register
            return;
        }
        if (write_reg8(LEDOUT1_REG, 0b10101010)) { // LED driver x individual brightness can be controlled through its PWMx register
            return;
        }

        for (auto& e : pwm_dt_) {
            e = 0xff;
        }

        init_ = true;
    }

    bool update(const uint8_t led, const uint8_t value) const {
        if (!init_ || led > 8) {
            return false;
        }

        return write_reg8(led + 2, value) == 0;
    }

public:
    /**
     * @return Current LED setting as bitmask
     */
    auto get() const {
        return status_;
    }

    auto get_init() const {
        return init_;
    }

    void set_pwm(const T leds, const uint8_t pwm) {
        uint8_t i { 1 };
        for (auto& e : pwm_dt_) {
            if (static_cast<uint8_t>(leds) & i) {
                e = pwm;
            }
            i <<= 1;
        }
    }

    /**
     * @brief Activate and deactivate LEDs as given by a bitmask
     * @param[in] leds: Bitmask for LEDs to set
     */
    bool set(const T leds) {
        if (!init_) {
            return false;
        }

        bool ret { true };
        for (uint8_t i {}; i < 8; ++i) {
            if ((static_cast<uint8_t>(status_) & (1 << i)) == (static_cast<uint8_t>(leds) & (1 << i))) {
                continue;
            }

            ret &= update(i, static_cast<uint8_t>(leds) & (1 << i) ? pwm_dt_[i] : 0); // FIXME: burst write?
            if (!ret) {
                break;
            }
        }

        status_ = leds;

        return ret;
    }

    /**
     * @brief Activate additional LEDs as given by an enable bitmask
     * @param[in] leds: Bitmask for LEDs to activate (others are not affected)
     */
    bool on(const T leds) {
        return set(status_ | leds);
    }

    /**
     * @brief Deactivate LEDs as given by a disable bitmask
     * @param[in] leds: Bitmask for LEDs to deactivate (others are not affected)
     */
    bool off(const T leds) {
        return set(status_ & ~leds);
    }
};

/**
 * @brief LED driver
 *
 * @startuml{LedsI2c.png}
 *  !include ledsi2c.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class LedsI2c : public LedsI2cBase<LedTypes> {
public:
    /**
     * @brief Construct a new Leds object
     */
    LedsI2c(I2C_Service* p_i2c_svc, const uint8_t i2c_addr) : LedsI2cBase { p_i2c_svc, i2c_addr } {}
};

/**
 * @brief LED driver
 *
 * @startuml{LedsI2cEna.png}
 *  !include ledsi2c.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
template <uint32_t HW_REV = CtBotConfig::MAINBOARD_REVISION>
class LedsI2cEna : public LedsI2cBase<LedTypesEna<HW_REV>> {
public:
    /**
     * @brief Construct a new Leds object
     */
    LedsI2cEna(I2C_Service* p_i2c_svc, const uint8_t i2c_addr) : LedsI2cBase<LedTypesEna<HW_REV>> { p_i2c_svc, i2c_addr } {}
};

} // namespace ctbot

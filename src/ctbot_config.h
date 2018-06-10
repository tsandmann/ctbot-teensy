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
 * @file    ctbot_config.h
 * @brief   Configuration settings
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#ifndef SRC_CTBOT_CONFIG_H_
#define SRC_CTBOT_CONFIG_H_

#include <cstdint>
#include <type_traits>


namespace ctbot {

/**
 * @brief Configuration settings
 *
 * @startuml{CtBotConfig.png}
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 *  class ctbot::CtBotConfig {
 *      +{static} BLINK_TEST_AVAILABLE : constexpr bool
 *      +{static} LED_TEST_AVAILABLE : constexpr bool
 *      +{static} ENA_TEST_AVAILABLE : constexpr bool
 *      +{static} LCD_TEST_AVAILABLE : constexpr bool
 *      +{static} SENS_LCD_TEST_AVAILABLE : constexpr bool
 *      +{static} UART0_BAUDRATE : constexpr uint32_t = 4000000UL
 *      +{static} RC5_ADDR : constexpr uint8_t = 6
 *  }
 * @enduml
 */
class CtBotConfig {
public:
    /* test tasks */
    static constexpr bool BLINK_TEST_AVAILABLE { false }; /**< Statically activate or deactivate LED blink test */
    static constexpr bool LED_TEST_AVAILABLE { true }; /**< Statically activate or deactivate LED test */
    static constexpr bool ENA_TEST_AVAILABLE { false }; /**< Statically activate or deactivate ENA test */
    static constexpr bool LCD_TEST_AVAILABLE { false }; /**< Statically activate or deactivate LCD test */
    static constexpr bool SENS_LCD_TEST_AVAILABLE { true }; /**< Statically activate or deactivate sensor display test */

    /* uart */
    static constexpr uint32_t UART0_BAUDRATE { 4000000UL }; /**< Baud rate used for Uart 0 (USB) */
    static constexpr uint8_t UART5_PIN_RX { 34U }; /**< Number of pin to use for RX line for Uart 5 */
    static constexpr uint8_t UART5_PIN_TX { 33U }; /**< Number of pin to use for TX line for Uart 5 */
    static constexpr uint32_t UART5_BAUDRATE { 115200UL }; /**< Baud rate used for Uart 5 */
    static constexpr uint8_t UART_FOR_CMD { 0 }; /**< Number of UART to use for command line interface */

    /* shift */
    static constexpr uint8_t SHIFT_SDATA_PIN { 43 }; /**< Pin number of shift register SDATA signal */

    /* ena */
    static constexpr uint8_t ENA_SCK_PIN { 41 }; /**< Pin number of ena shift register SCK signal */
    static constexpr uint8_t ENA_RCK_PIN { 42 }; /**< Pin number of ena shift register RCK signal */

    /* leds */
    static constexpr uint8_t LED_SCK_PIN { 40 }; /**< Pin number of led shift register SCK signal */
    static constexpr uint8_t LED_RCK_PIN { 42 }; /**< Pin number of led shift register RCK signal */

    /* KLAPPE */
    static constexpr uint8_t SHUTTER_PIN { 55 }; /**< Pin number of KLAPPE signal */

    /* SCHRANKE */
    static constexpr uint8_t TRANSPORT_PIN { 54 }; /**< Pin number of SCHRANKE signal */

    /* ENC_L */
    static constexpr uint8_t ENC_L_PIN { 24 }; /**< Pin number of ENC_L signal */

    /* ENC_R */
    static constexpr uint8_t ENC_R_PIN { 25 }; /**< Pin number of ENC_R signal */

    /* FERNBED */
    static constexpr uint8_t RC5_PIN { 28 }; /**< Pin number of RC5 signal */
    static constexpr uint8_t RC5_ADDR { 6 }; /**< RC5 address of used remote control */

    /* analog signals */
    static constexpr uint8_t DISTANCE_L_PIN { 65 }; /**< Pin number of DISTANCE_L signal */
    static constexpr uint8_t DISTANCE_R_PIN { 64 }; /**< Pin number of DISTANCE_R signal */
    static constexpr uint8_t LINE_L_PIN { 18 }; /**< Pin number of LINE_L signal */
    static constexpr uint8_t LINE_R_PIN { 19 }; /**< Pin number of LINE_R signal */
    static constexpr uint8_t LDR_L_PIN { 37 }; /**< Pin number of LDR_L signal */
    static constexpr uint8_t LDR_R_PIN { 38 }; /**< Pin number of LDR_R signal */
    static constexpr uint8_t BORDER_L_PIN { 16 }; /**< Pin number of BORDER_L signal */
    static constexpr uint8_t BORDER_R_PIN { 17 }; /**< Pin number of BORDER_R signal */

    /* motors */
    static constexpr uint8_t MOT_L_PWM_PIN { 35 }; /**< Pin number of left motor pwm signal */
    static constexpr uint8_t MOT_L_DIR_PIN { 29 }; /**< Pin number of left motor direction signal */
    static constexpr uint8_t MOT_R_PWM_PIN { 36 }; /**< Pin number of right motor pwm signal */
    static constexpr uint8_t MOT_R_DIR_PIN { 30 }; /**< Pin number of right motor direction signal */
    static constexpr int16_t MOT_PWM_MAX { 16000 }; /**< Maximum pwm duty cycle value for motors */

    /* servos */
    static constexpr uint8_t SERVO_1_PIN { 5 }; /**< Pin number of servo 1 pwm signal */
    static constexpr uint8_t SERVO_2_PIN { 6 }; /**< Pin number of servo 2 pwm signal */
};

} /* namespace ctbot */

#endif /* SRC_CTBOT_CONFIG_H_ */

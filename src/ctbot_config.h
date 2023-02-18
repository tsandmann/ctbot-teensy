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
 * @file    ctbot_config.h
 * @brief   Configuration settings
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include <type_traits>
#include <cstdint>


namespace ctbot {

template <uint32_t>
struct CtBotConfigBoard;

struct CtBotConfigBase {
    static constexpr bool IS_SIMULATED {
#ifdef CTBOT_SIMULATION
        true
#else
        false
#endif
    };
};

/**
 * @brief User configuration settings
 *
 * @startuml{CtBotConfigUser.png}
 *  !include ctbot_config.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
struct CtBotConfigUser : public CtBotConfigBase {
    static constexpr bool BEHAVIOR_MODEL_AVAILABLE { true };
    static constexpr bool BEHAVIOR_LEGACY_SUPPORT_AVAILABLE { true };

    /* test tasks */
    static constexpr bool BLINK_TEST_AVAILABLE { false }; /**< Statically activate or deactivate LED blink test */
    static constexpr bool LED_TEST_AVAILABLE { false }; /**< Statically activate or deactivate LED test */
    static constexpr bool ENA_TEST_AVAILABLE { false }; /**< Statically activate or deactivate ENA test */
    static constexpr bool LCD_TEST_AVAILABLE { false }; /**< Statically activate or deactivate LCD test */
    static constexpr bool SENS_LCD_TEST_AVAILABLE { false }; /**< Statically activate or deactivate sensor display test */
    static constexpr bool TASKWAIT_TEST_AVAILABLE { false }; /**< Statically activate or deactivate task wait test */
    static constexpr bool TFT_TEST_AVAILABLE { false }; /**< Statically activate or deactivate TFT test */
    static constexpr bool TOUCH_TEST_AVAILABLE { false }; /**< Statically activate or deactivate touchscreen test */
    static constexpr bool BUTTON_TEST_AVAILABLE { false }; /**< Statically activate or deactivate touchscreen button test */
    static constexpr bool AUDIO_TEST_AVAILABLE { false }; /**< Statically activate or deactivate audio sine wave test */

    /* features/modules available */
    static constexpr bool LCD_AVAILABLE { false }; /**< Statically activate or deactivate LC display */
    static constexpr bool TFT_AVAILABLE { false }; /**< Statically activate or deactivate TFT display */
    static constexpr bool FXA_FXO_AVAILABLE { false };
    static constexpr bool GYRO_AVAILABLE { false };
    static constexpr bool ACCEL_AVAILABLE { false };
    static constexpr bool MPU6050_AVAILABLE { false };
    static constexpr bool AUDIO_ANALOG_AVAILABLE { false }; /**< Statically activate or deactivate analog audio output */
    static constexpr bool AUDIO_I2S_AVAILABLE { false }; /**< Statically activate or deactivate I2S audio output */
    static constexpr bool SDCARD_AVAILABLE { false };
    static constexpr bool LOG_TO_SDCARD_AVAILABLE { false };
    static constexpr bool CLI_HISTORY_ON_SDCARD_AVAILABLE { false };
    static constexpr bool PROG_AVAILABLE { true }; /**< Statically activate or deactivate script execution features */
    static constexpr bool LUA_AVAILABLE { false }; /**< Statically activate or deactivate LUA interpreter */
    static constexpr bool I2C_TOOLS_AVAILABLE { true }; /**< Statically activate or deactivate i2c console commands */
    static constexpr bool DATE_TIME_AVAILABLE { true };

    static constexpr uint32_t BOOT_DELAY_MS { 2'000 };

    /* HW setup */
    static constexpr uint32_t MAINBOARD_REVISION { 9'002 }; // v0.9.0 or v0.9.2
    static constexpr uint32_t VOLTAGE_MEASURE_OFFSETS[2] { 0, 300 }; /**< Offsets for battery measurement voltage devider in mOhm */

    /* uart */
    static constexpr uint32_t UART0_BAUDRATE { 4'000'000 }; /**< Baud rate used for Uart 0 (USB) */
    static constexpr uint32_t UART_WIFI_BAUDRATE { 1'000'000 }; /**< Baud rate used for Uart to WiFi */
    static constexpr uint32_t UART_MOTOR_CTL_BAUDRATE { 1'000'000 }; /**< Baud rate used for motor Uart */
    static constexpr bool UART_WIFI_FOR_CMD { false }; /**< Use wifi for command line interface? */

    /* i2c */
    static constexpr uint32_t I2C1_FREQ { 400'000 };
    static constexpr uint32_t I2C2_FREQ { 400'000 };
    static constexpr uint32_t I2C3_FREQ { 100'000 };

    /* SD card */
    static constexpr bool SDCARD_USE_DMA { true };

    /* FERNBED */
    static constexpr uint8_t RC5_ADDR { 6 }; /**< RC5 address of used remote control */

    /* motors */
    static constexpr bool MOT_L_DIR { false }; /**< Direction of rotation for left motor */
    static constexpr bool MOT_R_DIR { false }; /**< Direction of rotation for right motor */

    /* audio output channels */
    static constexpr uint8_t AUDIO_CHANNELS { 2 };

    /* tft */
    static constexpr uint16_t TFT_CONTROLLER_TYPE { 9486 }; /**< 9486 for ILI9486 or 9341 for ILI9341 */
    static constexpr uint32_t TFT_SPI_FREQUENCY { 35'000'000 };
    static constexpr uint8_t TFT_ROTATION { 1 };
    static constexpr uint8_t TFT_TOUCH_ROTATION { 4 };
    static constexpr float TFT_BACKLIGHT_LEVEL { 5 };

    /* External RAM */
    static constexpr uint32_t PSRAM_FREQUENCY_MHZ { 132 }; // default: 88 MHz; valid: 66, 76, 88, 106, 132 MHz

    static constexpr float WHEEL_PERIMETER { 178.1283 }; // mm
    static constexpr float WHEEL_TO_WHEEL_DISTANCE { 97.2 }; // mm
};

template <uint32_t VERSION>
struct CtBotConfigHardware {
    static_assert(VERSION == 9'000 || VERSION == 9'002, "Invalid board configured, check CtBotConfigUser::MAINBOARD_REVISION");
};

/*
 * Config for mainboard version 0.9.0
 */
template <>
struct CtBotConfigHardware<9'000> {
    static constexpr uint8_t DEBUG_LED_PIN { 8 }; /**< Pin number of debug LED */

    /* uart */
    static constexpr uint8_t UART_WIFI { 7 }; /**< ID of UART to use for WiFi interface */
    static constexpr uint8_t UART_WIFI_PIN_RX { 28 }; /**< Number of pin to use for RX line for Uart to WiFi */
    static constexpr uint8_t UART_WIFI_PIN_TX { 29 }; /**< Number of pin to use for TX line for Uart to WiFi */

    static constexpr uint8_t UART_MOTOR_CTL { 6 }; /**< ID of UART to use for external motor control board */
    static constexpr uint8_t UART_MOTOR_CTL_PIN_RX { 25 }; /**< Number of pin to use for RX line for motor Uart */
    static constexpr uint8_t UART_MOTOR_CTL_PIN_TX { 24 }; /**< Number of pin to use for TX line for motor Uart */

    /* i2c */
    static constexpr uint8_t I2C1_PIN_SCL { 19 }; /**< Pin number of SCL for I2C 1 */
    static constexpr uint8_t I2C1_PIN_SDA { 18 }; /**< Pin number of SDA for I2C 1 */

    static constexpr uint8_t I2C2_PIN_SCL { 16 }; /**< Pin number of SCL for I2C 2 */
    static constexpr uint8_t I2C2_PIN_SDA { 17 }; /**< Pin number of SDA for I2C 2 */

    static constexpr uint8_t I2C3_PIN_SCL { 255 }; /**< Pin number of SCL for I2C 3 */
    static constexpr uint8_t I2C3_PIN_SDA { 255 }; /**< Pin number of SDA for I2C 3 */

    /* spi */
    static constexpr uint8_t SPI1_PIN_MOSI { 11 }; /**< Pin number of MOSI for SPI 1 */
    static constexpr uint8_t SPI1_PIN_MISO { 12 }; /**< Pin number of MISO for SPI 1 */
    static constexpr uint8_t SPI1_PIN_SCK { 13 }; /**< Pin number of SCK for SPI 1 */
    static constexpr uint8_t SPI1_PIN_CS1 { 10 }; /**< Pin number of default CS (CS1) for SPI 1 */
    static constexpr uint8_t SPI1_PIN_CS2 { 36 }; /**< Pin number of CS2 for SPI 1 */
    static constexpr uint8_t SPI1_PIN_CS3 { 37 }; /**< Pin number of CS3 for SPI 1 */

    static constexpr uint8_t SPI2_PIN_MOSI { 1 }; /**< Pin number of MOSI for SPI 2 */
    static constexpr uint8_t SPI2_PIN_MISO { 26 }; /**< Pin number of MISO for SPI 2 */
    static constexpr uint8_t SPI2_PIN_SCK { 27 }; /**< Pin number of SCK for SPI 2 */
    static constexpr uint8_t SPI2_PIN_CS { 0 }; /**< Pin number of default CS for SPI 2 */

    /* RC5 */
    static constexpr uint8_t RC5_PIN { 32 }; /**< Pin number of RC5 signal */

    /* BPS */
    static constexpr uint8_t BPS_PIN { 33 }; /**< Pin number of BPS signal */

    /* analog signals */
    static constexpr uint8_t LINE_L_PIN { 41 }; /**< Pin number of LINE_L signal */
    static constexpr uint8_t LINE_R_PIN { 40 }; /**< Pin number of LINE_R signal */
    static constexpr uint8_t BORDER_L_PIN { 39 }; /**< Pin number of BORDER_L signal */
    static constexpr uint8_t BORDER_R_PIN { 38 }; /**< Pin number of BORDER_R signal */
    static constexpr uint8_t BAT_VOLTAGE_PIN { 15 }; /**< Pin number of battery voltage signal */
    static constexpr uint8_t SERVO_1_FB_PIN { 22 }; /**< Pin number of servo 1 feedback signal */
    static constexpr uint8_t SERVO_2_FB_PIN { 255 }; /**< Pin number of servo 2 feedback signal */
    static constexpr uint8_t CURRENT_5V_PIN { 255 }; /**< Pin number of 5V current measurement signal */
    static constexpr uint8_t CURRENT_SERVO_PIN { 255 }; /**< Pin number of servo current measurement signal */

    /* motors */
    static constexpr bool EXTERNAL_SPEEDCTRL { false };
    static constexpr uint8_t MOT_L_PWM_PIN { 2 }; /**< Pin number of left motor pwm signal */
    static constexpr uint8_t MOT_L_DIR_PIN { 6 }; /**< Pin number of left motor direction signal */
    static constexpr uint8_t MOT_R_PWM_PIN { 3 }; /**< Pin number of right motor pwm signal */
    static constexpr uint8_t MOT_R_DIR_PIN { 9 }; /**< Pin number of right motor direction signal */
    static constexpr int16_t MOT_PWM_MAX { 16'000 }; /**< Maximum pwm duty cycle value for motors */

    /* wheel encoder */
    static constexpr uint8_t ENC_L_PIN { 30 }; /**< Pin number of ENC_L signal */
    static constexpr uint8_t ENC_R_PIN { 31 }; /**< Pin number of ENC_R signal */
    static constexpr uint16_t ENCODER_MARKS { 160 }; /**< Number of encoder marks on a wheel */

    /* PWM */
    static constexpr uint8_t PWM_ERW_1_PIN { 14 }; /**< Pin number of extension 1 pwm signal */
    static constexpr uint8_t PWM_ERW_2_PIN { 5 }; /**< Pin number of extension 1 pwm signal */

    /* servos */
    static constexpr uint8_t SERVO_1_PIN { 4 }; /**< Pin number of servo 1 pwm signal */
    static constexpr uint8_t SERVO_2_PIN { 23 }; /**< Pin number of servo 2 pwm signal */

    /* GPIO */
    static constexpr uint8_t GPIO_1_PIN { 24 }; /**< Pin number of GPIO 1 signal */
    static constexpr uint8_t GPIO_2_PIN { 25 }; /**< Pin number of GPIO 2 signal */

    /* lcd */
    static constexpr uint8_t LCD_I2C_BUS { 1 }; /**< ID of I2C bus to use for lcd controller */

    /* tft */
    static constexpr uint8_t TFT_SPI { 1 }; /**< ID of SPI port to use for tft controller */
    static constexpr uint8_t TFT_CS_PIN { SPI1_PIN_CS1 }; /**< Pin number of SPI chip select signal to use for tft controller */
    static constexpr uint8_t TFT_DC_PIN { SPI1_PIN_CS3 }; /**< Pin number of data/control signal to use for tft controller */
    static constexpr uint8_t TFT_BACKLIGHT_PIN { PWM_ERW_2_PIN }; /**< Pin number of backlight PWM control signal to use for tft display */
    static constexpr uint8_t TFT_TOUCH_CS_PIN { SPI1_PIN_CS2 }; /**< Pin number of SPI chip select signal to use for touch controller */

    /* ena i2c */
    static constexpr uint8_t ENA_I2C_BUS { 1 }; /**< ID of I2C port to use for ena controller */
    static constexpr uint8_t ENA_I2C_ADDR { 0b010'0000 /* 0x20 */ }; /**< I2C address of ena controller */
    static constexpr uint8_t ENA_PWM_I2C_BUS { 1 }; /**< Number of I2C port to use for ena-pwm controller */
    static constexpr uint8_t ENA_PWM_I2C_ADDR { 0b001'1101 /* 0x1d */ }; /**< I2C address of ena-pwm controller */

    /* leds i2c */
    static constexpr uint8_t LED_I2C_BUS { 1 }; /**< ID of I2C port to use for led controller */
    static constexpr uint8_t LED_I2C_ADDR { 0b001'0101 /* 0x15 */ }; /**< I2C address of led controller */

    /* VL53L0X */
    static constexpr uint8_t VL53L0X_I2C_BUS { 1 }; /**< ID of I2C port to use for VL53L0X sensors */
    static constexpr uint8_t VL53L0X_L_I2C_ADDR { 0x70 }; /**< I2C address of left VL53L0X sensor */
    static constexpr uint8_t VL53L0X_R_I2C_ADDR { 0x71 }; /**< I2C address of right VL53L0X sensor */

    /* VL6180X */
    static constexpr uint8_t VL6180X_I2C_BUS { 1 }; /**< ID of I2C port to use for VL6180X sensor */
    static constexpr uint8_t VL6180X_I2C_ADDR { 0x72 }; /**< I2C address of VL6180X sensor */

    /* IMU */
    static constexpr uint8_t IMU_I2C_BUS { 1 }; /**< ID of I2C port to use for IMU sensor */
};

/*
 * Config for mainboard version 0.9.2
 */
template <>
struct CtBotConfigHardware<9'002> {
    static constexpr uint8_t DEBUG_LED_PIN { 9 }; /**< Pin number of debug LED */

    /* uart */
    static constexpr uint8_t UART_WIFI { 8 }; /**< ID of UART to use for WiFi interface */
    static constexpr uint8_t UART_WIFI_PIN_RX { 34 }; /**< Number of pin to use for RX line for Uart to WiFi */
    static constexpr uint8_t UART_WIFI_PIN_TX { 35 }; /**< Number of pin to use for TX line for Uart to WiFi */

    static constexpr uint8_t UART_MOTOR_CTL { 2 }; /**< ID of UART to use for external motor control board */
    static constexpr uint8_t UART_MOTOR_CTL_PIN_RX { 7 }; /**< Number of pin to use for RX line for motor Uart */
    static constexpr uint8_t UART_MOTOR_CTL_PIN_TX { 8 }; /**< Number of pin to use for TX line for motor Uart */

    /* i2c */
    static constexpr uint8_t I2C1_PIN_SCL { 19 }; /**< Pin number of SCL for I2C 1 */
    static constexpr uint8_t I2C1_PIN_SDA { 18 }; /**< Pin number of SDA for I2C 1 */

    static constexpr uint8_t I2C2_PIN_SCL { 16 }; /**< Pin number of SCL for I2C 2 */
    static constexpr uint8_t I2C2_PIN_SDA { 17 }; /**< Pin number of SDA for I2C 2 */

    static constexpr uint8_t I2C3_PIN_SCL { 24 }; /**< Pin number of SCL for I2C 3 */
    static constexpr uint8_t I2C3_PIN_SDA { 25 }; /**< Pin number of SDA for I2C 3 */

    /* spi */
    static constexpr uint8_t SPI1_PIN_MOSI { 11 }; /**< Pin number of MOSI for SPI 1 */
    static constexpr uint8_t SPI1_PIN_MISO { 12 }; /**< Pin number of MISO for SPI 1 */
    static constexpr uint8_t SPI1_PIN_SCK { 13 }; /**< Pin number of SCK for SPI 1 */
    static constexpr uint8_t SPI1_PIN_CS1 { 10 }; /**< Pin number of default CS (CS1) for SPI 1 */
    static constexpr uint8_t SPI1_PIN_CS2 { 36 }; /**< Pin number of CS2 for SPI 1 */
    static constexpr uint8_t SPI1_PIN_CS3 { 37 }; /**< Pin number of CS3 for SPI 1 */

    static constexpr uint8_t SPI2_PIN_MOSI { 1 }; /**< Pin number of MOSI for SPI 2 */
    static constexpr uint8_t SPI2_PIN_MISO { 26 }; /**< Pin number of MISO for SPI 2 */
    static constexpr uint8_t SPI2_PIN_SCK { 27 }; /**< Pin number of SCK for SPI 2 */
    static constexpr uint8_t SPI2_PIN_CS { 0 }; /**< Pin number of default CS for SPI 2 */

    /* RC5 */
    static constexpr uint8_t RC5_PIN { 30 }; /**< Pin number of RC5 signal */

    /* BPS */
    static constexpr uint8_t BPS_PIN { 31 }; /**< Pin number of BPS signal */

    /* analog signals */
    static constexpr uint8_t LINE_L_PIN { 41 }; /**< Pin number of LINE_L signal */
    static constexpr uint8_t LINE_R_PIN { 40 }; /**< Pin number of LINE_R signal */
    static constexpr uint8_t BORDER_L_PIN { 39 }; /**< Pin number of BORDER_L signal */
    static constexpr uint8_t BORDER_R_PIN { 38 }; /**< Pin number of BORDER_R signal */
    static constexpr uint8_t BAT_VOLTAGE_PIN { 20 }; /**< Pin number of battery voltage signal */
    static constexpr uint8_t CURRENT_5V_PIN { 14 }; /**< Pin number of 5V current measurement signal */
    static constexpr uint8_t CURRENT_SERVO_PIN { 15 }; /**< Pin number of servo current measurement signal */
    static constexpr uint8_t SERVO_1_FB_PIN { 21 }; /**< Pin number of servo 1 feedback signal */
    static constexpr uint8_t SERVO_2_FB_PIN { 22 }; /**< Pin number of servo 2 feedback signal */
    static constexpr uint8_t SERVO_3_FB_PIN { 23 }; /**< Pin number of servo 3 feedback signal */

    /* motors */
    static constexpr bool EXTERNAL_SPEEDCTRL { true };

    /* wheel encoder */
    static constexpr uint16_t ENCODER_MARKS { 1'800 }; /**< Number of encoder marks on a wheel */

    /* servos */
    static constexpr uint8_t SERVO_1_PIN { 5 }; /**< Pin number of servo 1 pwm signal */
    static constexpr uint8_t SERVO_2_PIN { 6 }; /**< Pin number of servo 2 pwm signal */
    static constexpr uint8_t SERVO_3_PIN { 28 }; /**< Pin number of servo 2 pwm signal */
    static constexpr uint8_t SERVO_4_PIN { 29 }; /**< Pin number of servo 2 pwm signal */

    /* GPIO */
    static constexpr uint8_t GPIO_1_PIN { 32 }; /**< Pin number of GPIO 1 signal */
    static constexpr uint8_t GPIO_2_PIN { 33 }; /**< Pin number of GPIO 2 signal */

    /* lcd */
    static constexpr uint8_t LCD_I2C_BUS { 1 }; /**< ID of I2C bus to use for lcd controller */

    /* tft */
    static constexpr uint8_t TFT_SPI { 1 }; /**< ID of SPI port to use for tft controller */
    static constexpr uint8_t TFT_CS_PIN { SPI1_PIN_CS1 }; /**< Pin number of SPI chip select signal to use for tft controller */
    static constexpr uint8_t TFT_DC_PIN { SPI1_PIN_CS3 }; /**< Pin number of data/control signal to use for tft controller */
    static constexpr uint8_t TFT_BACKLIGHT_PIN { 255 }; /**< Pin number of backlight PWM control signal to use for tft display */ // FIXME: DRV_DISPLAY
    static constexpr uint8_t TFT_TOUCH_CS_PIN { SPI1_PIN_CS2 }; /**< Pin number of SPI chip select signal to use for touch controller */

    /* ena i2c */
    static constexpr uint8_t ENA_I2C_BUS { 1 }; /**< ID of I2C port to use for ena controller */
    static constexpr uint8_t ENA_I2C_ADDR { 0b010'0000 /* 0x20 */ }; /**< I2C address of ena controller */
    static constexpr uint8_t ENA_PWM_I2C_BUS { 1 }; /**< Number of I2C port to use for ena-pwm controller */
    static constexpr uint8_t ENA_PWM_I2C_ADDR { 0b001'1101 /* 0x1d */ }; /**< I2C address of ena-pwm controller */

    /* leds i2c */
    static constexpr uint8_t LED_I2C_BUS { 1 }; /**< ID of I2C port to use for led controller */
    static constexpr uint8_t LED_I2C_ADDR { 0b001'0101 /* 0x15 */ }; /**< I2C address of led controller */

    /* VL53L0X */
    static constexpr uint8_t VL53L0X_I2C_BUS { 1 }; /**< ID of I2C port to use for VL53L0X sensors */
    static constexpr uint8_t VL53L0X_L_I2C_ADDR { 0x70 }; /**< I2C address of left VL53L0X sensor */
    static constexpr uint8_t VL53L0X_R_I2C_ADDR { 0x71 }; /**< I2C address of right VL53L0X sensor */

    /* VL6180X */
    static constexpr uint8_t VL6180X_I2C_BUS { 1 }; /**< ID of I2C port to use for VL6180X sensor */
    static constexpr uint8_t VL6180X_I2C_ADDR { 0x72 }; /**< I2C address of VL6180X sensor */

    /* IMU */
    static constexpr uint8_t IMU_I2C_BUS { 2 }; /**< ID of I2C port to use for IMU sensor */

    /* unused settings for this configuration */
    static constexpr uint8_t MOT_L_PWM_PIN { 255 }; /**< Pin number of left motor pwm signal */
    static constexpr uint8_t MOT_L_DIR_PIN { 255 }; /**< Pin number of left motor direction signal */
    static constexpr uint8_t MOT_R_PWM_PIN { 255 }; /**< Pin number of right motor pwm signal */
    static constexpr uint8_t MOT_R_DIR_PIN { 255 }; /**< Pin number of right motor direction signal */
    static constexpr int16_t MOT_PWM_MAX { 0 }; /**< Maximum pwm duty cycle value for motors */
    static constexpr uint8_t ENC_L_PIN { 255 }; /**< Pin number of ENC_L signal */
    static constexpr uint8_t ENC_R_PIN { 255 }; /**< Pin number of ENC_R signal */
};

template <class T>
struct CtBotConfigHelper : public T, public CtBotConfigUser {
    static constexpr bool AUDIO_AVAILABLE { CtBotConfigUser::AUDIO_ANALOG_AVAILABLE
        ^ CtBotConfigUser::AUDIO_I2S_AVAILABLE }; /**< Statically activate or deactivate audio features */
    static constexpr uint8_t AUDIO_MEMORY_BLOCKS { AUDIO_AVAILABLE ? 12 : 0 };
    static constexpr uint16_t ENCODER_MARKS { CtBotConfigBase::IS_SIMULATED ? 160 : T::ENCODER_MARKS }; /**< Number of encoder marks on a wheel */
    static constexpr bool EXTERNAL_SPEEDCTRL { T::EXTERNAL_SPEEDCTRL && (!CtBotConfigBase::IS_SIMULATED) };
};

using CtBotConfig = CtBotConfigHelper<CtBotConfigHardware<CtBotConfigUser::MAINBOARD_REVISION>>;

static_assert(CtBotConfig::AUDIO_CHANNELS > 0 && CtBotConfig::AUDIO_CHANNELS <= 2, "number of AUDIO_CHANNELS out of range [1; 2]");

} // namespace ctbot

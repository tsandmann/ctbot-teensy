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
 * @file    ctbot.h
 * @brief   Main class of ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "comm_interface.h"
#include "ctbot_config.h"
#include "driver/leds_i2c.h"

#include "arduino_freertos.h"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>


class AudioOutputAnalog;
class AudioStream;
class AudioPlaySdWav;
class AudioSynthWaveformSine;
class CtBotCli;
class TTS;
class AudioConnection;
class AudioMixer4;
class LuaWrapper;
class I2C_Service;
class FS_Service;

namespace freertos {
class SerialIO;
}

/**
 * @brief Namespace for all ct-Bot classes and functionality
 */
namespace ctbot {
class Sensors;
class Motor;
class SpeedControlBase;
class Servo;
class EnaI2c;
class LCDisplay;
class TFTDisplay;
class CmdParser;
class CtBotCli;
class Scheduler;
class SchedulerStatContext;
class ParameterStorage;
class Logger;
class LoggerTargetFile;

/**
 * @brief Main class of ct-Bot teensy framework, responsible for initialization and control loop execution
 */
class CtBot {
    static constexpr uint8_t DEBUG_LEVEL_ { 2 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose

    static consteval auto calc_flexspi2_divider(uint32_t freq_mhz) {
        return (freq_mhz <= 132 && freq_mhz >= 66) ? (528 + (freq_mhz - 1)) / freq_mhz - 1 : (freq_mhz < 66 ? 7 : 3);
    }

protected:
    friend class CtBotCli;
    static constexpr uint16_t TASK_PERIOD_MS_ { 10 }; /**< Scheduling period of task in ms */
    static constexpr uint8_t TASK_PRIORITY_ { 7 };
    static constexpr uint32_t STACK_SIZE_ { 2048 };
    static constexpr uint32_t VIEWER_SEND_INTERVAL_MS_ { 200 };
    static constexpr uint32_t VIEWER_SEND_TASKS_INTERVAL_MS_ { 2'000 };

    static TaskHandle_t audio_task_;

    bool shutdown_;
    bool ready_;
    uint16_t task_id_;
    Scheduler* p_scheduler_; /**< Pointer to scheduler instance */
    Sensors* p_sensors_; /**< Pointer to sensor instance */
    Motor* p_motors_[2]; /**< Pointer to motor instances */
    SpeedControlBase* p_speedcontrols_[2]; /**< Pointer to speed controller instances */
    Servo* p_servos_[2]; /**< Pointer to servo instances */
    I2C_Service* p_i2c_1_svc_;
    I2C_Service* p_i2c_2_svc_;
    EnaI2c* p_ena_;
    LedsI2cEna<>* p_ena_pwm_;
    LedsI2c* p_leds_; /**< Pointer to led instance */
    LCDisplay* p_lcd_; /**< Pointer to LC display instance */
    TFTDisplay* p_tft_; /**< Pointer to TFT display instance */
    CtBotCli* p_cli_;
    freertos::SerialIO* p_serial_usb_; /**< Pointer to serial connection abstraction layer instance for USB serial port*/
    freertos::SerialIO* p_serial_wifi_; /**< Pointer to serial connection abstraction layer instance for uart 5 (used for WiFi) */
    CommInterfaceCmdParser* p_comm_; /**< Pointer to (serial) communication interface instance */
    CmdParser* p_parser_; /**< Pointer to cmd parser instance */
    Logger* p_logger_;
    FS_Service* p_fs_;
    LoggerTargetFile* p_logger_file_;
    ParameterStorage* p_parameter_;
    AudioOutputAnalog* p_audio_output_dac_;
    AudioStream* p_audio_output_i2s_;
    AudioSynthWaveformSine* p_audio_sine_;
    AudioPlaySdWav* p_play_wav_;
    TTS* p_tts_;
    std::vector<AudioConnection*> p_audio_conn_;
    std::vector<AudioMixer4*> p_audio_mixer_;
    std::map<std::string, std::tuple<std::function<void()>, bool>, std::less<>> pre_hooks_;
    std::map<std::string, std::tuple<std::function<void()>, bool>, std::less<>> post_hooks_;
    TimerHandle_t p_watch_timer_;
    TimerHandle_t p_clock_timer_;
    bool clock_update_done_;
    LuaWrapper* p_lua_;
    uint32_t last_viewer_timestamp_;
    uint32_t last_taskstat_timestamp_;
    SchedulerStatContext* p_taskstat_context_;
    SchedulerStatContext* p_viewer_tasks_context_;
    TimerHandle_t shutdown_timer_;


    /**
     * @brief Constructor of main class
     * @note Constructor is protected to enforce singleton pattern
     * @details The constructor initializes usb serial port to allow early debug infos printed out there.
     */
    FLASHMEM CtBot();

    /* enforce singleton */
    CtBot(const CtBot&) = delete;
    void operator=(const CtBot&) = delete;
    CtBot(CtBot&&) = delete;

    /**
     * @brief Main task implementation
     * @note This method is run every TASK_PERIOD_MS ms, e.g. to update sensor data
     */
    virtual void run();

    /**
     * @brief Shut everything down and put the CPU into sleep mode
     * @details All motors, servos, leds, sensors are stopped / shut down and all tasks are suspended.
     */
    FLASHMEM virtual void shutdown();

    FLASHMEM void update_clock();

    bool create_shutdown_timer(uint32_t ms);

    bool publish_viewerdata(uint32_t now);

    FLASHMEM bool file_upload(freertos::SerialIO& io, const std::string_view& file_name, uint32_t file_size, bool textmode) const {
        return file_upload(io, file_name, file_size, 0, textmode);
    }

public:
    /**
     * @brief Get the one and only instance of this class (singleton)
     * @return Reference to CtBot instance
     */
    static CtBot& get_instance();

    bool get_ready() const {
        volatile bool ready { ready_ };
        return ready;
    }

    /**
     * @brief Destroy the CtBot instance
     */
    FLASHMEM virtual ~CtBot();

    /**
     * @brief Setup method responsible for initialization and creating of instances for all components (sensors, motors, etc.)
     * @param[in] set_ready: ready flag is set to this value at end of setup()
     */
    FLASHMEM virtual void setup(const bool set_ready);

    /**
     * @brief Stop the scheduler
     */
    FLASHMEM void stop();

    void set_clock_update_done(bool value) {
        clock_update_done_ = value;
    }

    bool play_wav(const std::string_view& filename);

    /**
     * @brief Get the sensor instance
     * @return Pointer to sensor instance
     */
    auto get_sensors() const {
        return p_sensors_;
    }

    /**
     * @brief Get the ena instance
     * @return Pointer to ena instance
     */
    auto get_ena() const {
        return p_ena_;
    }

    /**
     * @brief Get the ena_pwm instance
     * @return Pointer to ena_pwm instance
     */
    auto get_ena_pwm() const {
        return p_ena_pwm_;
    }

    /**
     * @brief Get the led instance
     * @return Pointer to led instance
     */
    auto get_leds() const {
        return p_leds_;
    }

    /**
     * @brief Get the LC display instance
     * @return Pointer to display instance
     */
    auto get_lcd() const {
        return p_lcd_;
    }

    auto get_tft() const {
        return p_tft_;
    }

    /**
     * @brief Get the speed controller instances of left and right motor
     * @return Pointer to speed controllers
     */
    auto get_speedcontrols() const {
        return p_speedcontrols_;
    }

    /**
     * @brief Get the servo controller intances
     * @return Pointer to servo controllers
     */
    auto get_servos() const {
        return p_servos_;
    }

    /**
     * @brief Get the scheduler instance
     * @return Pointer to scheduler instance
     */
    auto get_scheduler() const {
        return p_scheduler_;
    }

    /**
     * @brief Get the communication interface instance
     * @return Pointer to communication interface instance
     */
    auto get_comm() const {
        return p_comm_;
    }

    auto get_logger() const {
        return p_logger_;
    }

    auto get_cmd_parser() const {
        return p_parser_;
    }

    auto get_cli() const {
        return p_cli_;
    }

    /**
     * @brief Get the SerialIO instance of USB serial port
     * @return Pointer to SerialIO instance
     */
    auto get_serial_usb() const {
        return p_serial_usb_;
    }

    /**
     * @brief Get the SerialIO instance of command serial port
     * @return Pointer to SerialIO instance
     */
    auto get_serial_cmd() const {
        return CtBotConfig::UART_WIFI_FOR_CMD ? p_serial_wifi_ : p_serial_usb_;
    }

    auto get_fs() const {
        return p_fs_;
    }

    void add_pre_hook(const std::string& name, std::function<void()>&& hook, bool active = true);

    void add_post_hook(const std::string& name, std::function<void()>&& hook, bool active = true);

    FLASHMEM bool file_upload(freertos::SerialIO& io, const std::string_view& file_name, uint32_t file_size, uint32_t crc32, bool textmode) const;

    FLASHMEM uint32_t file_crc32(const std::string_view& file_name) const;
};

} // namespace ctbot

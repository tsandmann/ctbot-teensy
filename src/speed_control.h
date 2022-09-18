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
 * @file    speed_control.h
 * @brief   Motor speed controller
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "driver/encoder.h"
#include "driver/motor.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <list>


class Pid;

namespace arduino {
class SerialIO;
}

namespace ctbot {

class SpeedControlBase {
protected:
    friend class SimConnection;
    static constexpr uint16_t MAX_SPEED_ { 450 }; /**< Maximum possible speed in mm/s */

    float kp_, ki_, kd_;
    float setpoint_; // value in %

public:
    FLASHMEM SpeedControlBase();

    FLASHMEM virtual ~SpeedControlBase();

    /**
     * @brief Set a new desired speed
     * @param[in] speed: Speed to set as percentage value; [-100; +100]
     */
    virtual void set_speed(const float speed) = 0;

    /**
     * @return Current speed set
     */
    virtual float get_speed() const = 0;

    virtual float get_enc_speed() const = 0;

    virtual int32_t get_enc_counts() const = 0;

    /**
     * @return Current Kp parameter setting
     */
    auto get_kp() const {
        return kp_;
    }

    /**
     * @return Current Ki parameter setting
     */
    auto get_ki() const {
        return ki_;
    }

    /**
     * @return Current Kd parameter setting
     */
    auto get_kd() const {
        return kd_;
    }

    /**
     * @brief Set new parameters for underlying PID controller
     * @param[in] kp: Proportional tuning parameter
     * @param[in] ki: Integral tuning parameter
     * @param[in] kd: Derivative tuning parameter
     */
    virtual void set_parameters(const float kp, const float ki, const float kd) = 0;
};

/**
 * @brief Motor speed controller
 *
 * @startuml{SpeedControl.png}
 *  !include speed_control.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class SpeedControl : public SpeedControlBase {
protected:
    static constexpr uint16_t TASK_PERIOD_MS_ { 10 }; /**< Scheduling period of task in ms */
    static constexpr uint8_t TASK_PRIORITY_ { 8 };
    static constexpr uint32_t TASK_STACK_SIZE_ { 768 };

    static std::list<SpeedControl*> controller_list_; /**< List of all SpeedControl instances created */

    bool direction_;
    float input_, output_;
    Pid* p_pid_controller_;
    Encoder& wheel_encoder_;
    Motor& motor_;

    /**
     * @brief Perform the PID controller update step
     * @note Is called periodically by the speed controller task every TASK_PERIOD_MS ms
     */
    void run();

    /**
     * @brief Speed controller task implementation
     * @note Calls run() on all SpeedControl instances
     */
    static void controller();

public:
    /**
     * @brief Construct a new SpeedControl object
     * @param[in] wheel_enc: Reference to wheel encoder to use for controller input (speed)
     * @param[in] motor: Reference to motor driver to use for controller output (PWM duty cycle)
     */
    FLASHMEM SpeedControl(Encoder& wheel_enc, Motor& motor);

    /**
     * @brief Destroy the Speed Control object
     */
    FLASHMEM virtual ~SpeedControl();

    virtual void set_speed(const float speed) override { // FIXME: move to base class?
        auto abs_speed(std::fabs(speed));
        if (abs_speed > 100.f) {
            abs_speed = 100.f;
        }
        setpoint_ = abs_speed * (MAX_SPEED_ / 100.f); // speed is in %
        direction_ = speed >= 0.f;
    }

    virtual float get_speed() const override { // FIXME: move to base class
        return setpoint_ / (direction_ ? MAX_SPEED_ / 100.f : MAX_SPEED_ / -100.f); // convert speed to %
    }

    /**
     * @return Current wheel speed as reported by related wheel encoder
     */
    virtual float get_enc_speed() const override {
        return wheel_encoder_.get_speed();
    }

    virtual int32_t get_enc_counts() const override {
        return wheel_encoder_.get();
    }

    FLASHMEM virtual void set_parameters(const float kp, const float ki, const float kd) override;
};

class SpeedControlPico : public SpeedControlBase {
    static constexpr bool DEBUG_ { false };

    static std::list<SpeedControlPico*> controller_list_;
    static arduino::SerialIO& serial_;
    static uint16_t motor_current_;
    static uint32_t crc_errors_;

    float enc_speed_; // mm/s
    int32_t enc_counts_;
    int32_t last_counts_;
    bool reset_;

protected:
    struct EncData {
        const uint8_t start;
        int16_t rpm[2];
        int32_t counts[2];
        uint16_t motor_current;
        uint32_t crc;

        constexpr EncData() : start { 0xaa }, rpm {}, counts {}, motor_current {}, crc {} {}
    } __attribute__((__packed__));

    struct SpeedData {
        const uint8_t start;
        uint8_t cmd;
        float speed[2];
        uint32_t crc;

        constexpr SpeedData() : start { 0xaa }, cmd {}, speed {}, crc {} {}
    } __attribute__((__packed__));


    static constexpr uint16_t TASK_PERIOD_MS_ { 10 };
    static constexpr uint8_t TASK_PRIORITY_ { 8 };
    static constexpr uint32_t TASK_STACK_SIZE_ { 2048 };
    static constexpr size_t RX_BUF_SIZE_ { 8192 };
    static constexpr size_t TX_BUF_SIZE_ { 64 };

public:
    static void controller();

    static auto get_motor_current() {
        return motor_current_;
    }

    static auto get_crc_errors() {
        return crc_errors_;
    }

    FLASHMEM SpeedControlPico();

    FLASHMEM virtual ~SpeedControlPico();

    virtual void set_speed(const float speed) override { // FIXME: move to base class?
        setpoint_ = speed * 0.75f;
    }

    virtual float get_speed() const override { // FIXME: move to base class
        return setpoint_ / 0.75f;
    }

    virtual float get_enc_speed() const override {
        return enc_speed_;
    }

    virtual int32_t get_enc_counts() const override {
        return enc_counts_;
    }

    virtual void set_parameters(const float, const float, const float) override {
        return;
    }
};

using SpeedControlExternal = SpeedControlPico;

} // namespace ctbot

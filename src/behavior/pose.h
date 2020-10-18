/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    pose.h
 * @brief   Pose representation
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "speed.h"

#include <cstdint>
#include <cmath>
#include <memory>


namespace ctbot {
class CommInterface;

/**
 * @brief Pose representation
 *
 * @startuml{Pose.png}
 *  !include behavior/pose.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Pose {
protected:
    float x_; /**< X part of pose vector */
    float y_; /**< Y part of pose vector */
    float heading_; /**< Direction of pose vector */
    float heading_sin_; /**< Sine of direction (cached) */
    float heading_cos_; /**< Cosine of direction (cached) */

    /**
     * @brief Converts an angle in degree to radian measure
     *
     * @tparam T Type of input
     * @param[in] degree Angle in degree
     * @return Angle in radian measure
     */
    template <typename T>
    static constexpr T to_rad(const T degree) {
        return degree * static_cast<T>(M_PI / 180.);
    }

    /**
     * @brief Converts an angle in radian measure to degree
     *
     * @tparam T Type of input
     * @param[in] radian Angle in radian measure
     * @return Angle in degree
     */
    template <typename T>
    static constexpr T to_deg(const T radian) {
        return radian / static_cast<T>(M_PI / 180.);
    }

public:
    using Ptr = std::unique_ptr<Pose>;

    /**
     * @brief Construct a new Pose instance
     */
    Pose() noexcept : x_ {}, y_ {}, heading_ {}, heading_sin_ { std::sin(0.f) }, heading_cos_ { std::cos(0.f) } {}

    /**
     * @brief Construct a new Pose instance
     *
     * @param[in] x X part of pose vector
     * @param[in] y Y part of pose vector
     * @param heading Direction of pose vector
     */
    Pose(const float x, const float y, const float heading) noexcept
        : x_ { x }, y_ { y }, heading_ { heading }, heading_sin_ { std::sin(heading) }, heading_cos_ { std::cos(heading) } {}

    /**
     * @brief Get X part of pose vector
     *
     * @tparam T Type of return value
     * @return X part of pose in mm
     */
    template <typename T = float>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type get_x() const noexcept {
        return static_cast<T>(x_);
    }

    /**
     * @brief Set the X part of pose vector
     *
     * @tparam T Type of input
     * @param[in] value New value for X part of pose vector in mm
     */
    template <typename T>
    void set_x(const T& value) noexcept {
        x_ = static_cast<float>(value);
    }

    /**
     * @brief Get Y part of pose vector
     *
     * @tparam T Type of return value
     * @return Y part of pose in mm
     */
    template <typename T = float>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type get_y() const noexcept {
        return static_cast<T>(y_);
    }

    /**
     * @brief Set the Y part of pose vector
     *
     * @tparam T Type of input
     * @param[in] value New value for Y part of pose vector in mm
     */
    template <typename T>
    void set_y(const T& value) noexcept {
        y_ = static_cast<float>(value);
    }

    /**
     * @brief Get direction of pose vector
     *
     * @tparam T Type of return value
     * @return Heading of pose in degree
     */
    template <typename T = float>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type get_heading() const noexcept {
        return static_cast<T>(heading_);
    }

    /**
     * @brief Get the sine of direction of pose vector
     *
     * @tparam T Type of return value
     * @return Sine of heading of pose
     */
    template <typename T = float>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type get_heading_sin() const noexcept {
        return static_cast<T>(heading_sin_);
    }

    /**
     * @brief Get the cosine of direction of pose vector
     *
     * @tparam T Type of return value
     * @return Cosine of heading of pose
     */
    template <typename T = float>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type get_heading_cos() const noexcept {
        return static_cast<T>(heading_cos_);
    }

    /**
     * @brief Set direction of pose vector
     *
     * @tparam T Type of input
     * @param[in] value New value for heading in degree
     */
    template <typename T>
    void set_heading(const T& value) noexcept {
        heading_ = static_cast<float>(value);
        heading_sin_ = std::sin(to_rad(heading_));
        heading_cos_ = std::cos(to_rad(heading_));
    }

    /**
     * @brief Get the Euclidean distance between this pose and another
     *
     * @tparam T Type of return value
     * @param[in] other Referenz to the other pose
     * @return Square of Euclidean distance in mm
     */
    template <typename T = int32_t>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type get_dist_square(const Pose& other) const noexcept {
        const auto xt { other.x_ - this->x_ };
        const auto yt { other.y_ - this->y_ };

        /* Use Pythagoras' theorem to calc Euclidean distance */
        return static_cast<T>(xt * xt + yt * yt);
    }

    /**
     * @brief Get the turned angle between this pose and another
     *
     * @tparam T Type of return value
     * @param[in] other Referenz to the other pose
     * @param[in] bot_speed Referenz to the bot's speed during turning
     * @return Turned angle in degree
     */
    template <typename T = int16_t>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type turned_angle(const Pose& other, const Speed& bot_speed) const noexcept {
        if (bot_speed.get_left<int16_t>() == bot_speed.get_right<int16_t>()) {
            /* unknown direction of rotation */
            return 0;
        }
        T diff { get_heading<T>() - other.get_heading<T>() };
        diff = static_cast<T>(std::fmod(diff, 360.f));
        if (bot_speed.get_right<int16_t>() < bot_speed.get_left<int16_t>()) {
            /* negative direction of rotation */
            diff = static_cast<T>(-diff);
        }
        return diff;
    }

    /**
     * @brief Get the absolute value of the turned angle between this pose and another
     *
     * @tparam T Type of return value
     * @param[in] other Referenz to the other pose
     * @return Absolute value of turned angle in degree
     */
    template <typename T = int16_t>
    typename std::enable_if<std::is_fundamental<T>::value, const T>::type turned_angle_abs(const Pose& other) const noexcept {
        T diff { get_heading<T>() - other.get_heading<T>() };
        if (diff < static_cast<T>(0)) {
            diff = -diff;
        }
        diff = static_cast<T>(std::fmod(diff, 360.f));
        return diff;
    }

    /**
     * @brief Print pose out to a SerialConnection
     *
     * @param[in] comm Reference to communication interface to use
     */
    FLASHMEM void print(CommInterface& comm) const;
};

} // namespace ctbot

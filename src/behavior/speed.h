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
 * @file    speed.h
 * @brief   Speed representation
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include <cstdint>
#include <memory>


namespace ctbot {
class CommInterface;

/**
 * @brief Speed representation
 *
 * @startuml{Speed.png}
 *  !include behavior/speed.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Speed {
protected:
    using speed_t = int16_t;
    speed_t left_; /**< Speed of left wheel */
    speed_t right_; /**< Speed of right wheel */
    speed_t center_; /**< Speed of bot */

public:
    using Ptr = std::unique_ptr<Speed>;

    /**
     * @brief Construct a new Speed instance
     */
    Speed() noexcept : left_ {}, right_ {}, center_ {} {}

    /**
     * @brief Construct a new Speed instance
     *
     * @param[in] speed_l Initial speed value of left wheel
     * @param[in] speed_r Initial speed value of right wheel
     * @param[in] speed_center Initial speed value of bot
     */
    Speed(speed_t speed_l, speed_t speed_r, speed_t speed_center) noexcept : left_ { speed_l }, right_ { speed_r }, center_ { speed_center } {}

    /**
     * @brief Get speed of left wheel
     *
     * @tparam T Type of return value
     * @return Speed of left wheel in mm/s
     */
    template <typename T = speed_t>
    T get_left() const noexcept {
        return static_cast<T>(left_);
    }

    /**
     * @brief Set speed of left wheel
     *
     * @param[in] value New value for speed of left wheel in mm/s
     */
    void set_left(auto const& value) noexcept {
        left_ = static_cast<decltype(left_)>(value);
    }

    /**
     * @brief Get speed of right wheel
     *
     * @tparam T Type of return value
     * @return Speed of right wheel in mm/s
     */
    template <typename T = speed_t>
    T get_right() const noexcept {
        return static_cast<T>(right_);
    }

    /**
     * @brief Set speed of right wheel
     *
     * @param[in] value New value for speed of right wheel in mm/s
     */
    void set_right(auto const& value) noexcept {
        right_ = static_cast<decltype(right_)>(value);
    }

    /**
     * @brief Get speed of bot
     *
     * @tparam T Type of return value
     * @return Speed of bot in mm/s
     */
    template <typename T = speed_t>
    T get_center() const noexcept {
        return static_cast<T>(center_);
    }

    /**
     * @brief Set speed of bot
     *
     * @param[in] value New value for speed of bot in mm/s
     */
    void set_center(auto const& value) noexcept {
        center_ = static_cast<decltype(center_)>(value);
    }

    /**
     * @brief Print speed out to a SerialConnection
     *
     * @param[in] comm Reference to communication interface to use
     */
    FLASHMEM void print(CommInterface& comm) const;
};

} // namespace ctbot

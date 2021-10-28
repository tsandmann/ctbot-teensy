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
 * @file    resource.h
 * @brief   Abstraction for resources
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include <memory>
#include <list>
#include <tuple>
#include <functional>
#include <type_traits>


#if __cpp_exceptions
#define _try try
#define _catch(X) catch (X)
#define _throw_exception_again throw
#else
#define _try if (true)
#define _catch(X) if (false)
#define _throw_exception_again
#endif


namespace ctbot {

/**
 * @brief Base for all resource types
 *
 * @startuml{ResourceBase.png}
 *  !include behavior/resource.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class ResourceBase {
public:
    using Ptr = std::unique_ptr<ResourceBase>;
};

/**
 * @brief Resource abstraction
 *
 * @startuml{Resource.png}
 *  !include behavior/resource.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
template <typename T>
class Resource : public ResourceBase {
    // FIXME: add documentation
protected:
    T data_;
    std::list<std::tuple<std::function<void(const Resource&)>, void*>> listeners_;

public:
    using type = T;
    using basetype = Resource<T>;

    Resource() noexcept : data_() {}

    template <typename K = T>
    typename std::enable_if<std::is_fundamental<T>::value, const K>::type read() const noexcept {
        return static_cast<const K>(data_);
    }

    template <typename K = T>
    typename std::enable_if<!std::is_fundamental<T>::value, const K&>::type read() const noexcept {
        return static_cast<const K&>(data_);
    }

    template <typename K = T>
    bool write(typename std::enable_if<std::is_fundamental<T>::value, const K>::type x) noexcept {
        data_ = static_cast<const T>(x);
        return notify();
    }

    template <typename K = T>
    bool write(typename std::enable_if<!std::is_fundamental<T>::value, const K&>::type x) noexcept {
        data_ = static_cast<const T&>(x);
        return notify();
    }

    T& get_ref() noexcept {
        return data_;
    }

    bool notify() const noexcept {
        bool ret { true };
        for (auto& func : listeners_) {
            _try {
                std::get<0>(func)(*this);
            }
            _catch(...) {
                ret = false;
            }
        }
        return ret;
    }

    FLASHMEM void register_listener(std::function<void(const Resource&)> func, void* owner = nullptr) {
        listeners_.push_back(std::make_tuple(func, owner));
    }
};

} // namespace ctbot

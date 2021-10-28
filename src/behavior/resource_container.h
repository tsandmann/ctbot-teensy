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
 * @file    resource_container.h
 * @brief   Aggregation for resources
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "resource.h"

#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include <list>
#include <map>
#include <tuple>
#include <functional>
#include <mutex>


namespace ctbot {

/**
 * @brief Resource aggregation
 *
 * @startuml{ResourceContainer.png}
 *  !include behavior/resource_container.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class ResourceContainer : public ResourceBase { // FIXME: check locking!
protected:
    uint32_t next_id_;
    mutable std::mutex mutex_;
    std::map<const std::string /*name*/, std::tuple<uint32_t /*id*/, ResourceBase::Ptr /*resource*/>, std::less<>> resources_;
    std::vector<uint32_t> resources_active_;
    std::map<uint32_t /*id*/, bool /*updated*/> resource_updates_;
    std::list<std::tuple<std::function<void(const ResourceContainer&)>, void*>> listeners_;

    uint32_t get_id(const std::string_view& name) const {
        auto it { resources_.find(name) };
        if (it == resources_.end()) {
            return 0;
        }
        return std::get<0>(it->second);
    }

    FLASHMEM ResourceBase* get_resource_deep(const std::string_view& name) const;
    void call_listener() const;

public:
    using Ptr = std::unique_ptr<ResourceContainer>;

    FLASHMEM ResourceContainer() noexcept;
    ~ResourceContainer() = default;

    template <class T>
    FLASHMEM typename std::enable_if<!std::is_base_of<ResourceContainer, T>::value, Resource<T>*>::type create_resource(
        const std::string_view& container_name, const char* new_res, bool active, auto&&... args) {
        ResourceContainer* p_container { get_resource<ResourceContainer>(container_name) };
        if (p_container) {
            return p_container->create_resource<T>(new_res, active, args...);
        }
        return nullptr;
    }

    template <class T>
    FLASHMEM typename std::enable_if<!std::is_base_of<ResourceContainer, T>::value, Resource<T>*>::type create_resource(
        const std::string_view& name, bool active, auto&&... args) {
        std::unique_ptr<ResourceBase> p_res { std::make_unique<Resource<T>>(args...) };
        Resource<T>* ptr { static_cast<Resource<T>*>(p_res.get()) };
        if (add_resource(name, std::move(p_res), active)) {
            return ptr;
        }
        return nullptr;
    }

    template <class T>
    FLASHMEM typename std::enable_if<std::is_base_of<ResourceContainer, T>::value, T*>::type create_resource(
        const std::string_view& container_name, const char* new_res, bool active) {
        ResourceContainer* p_container { get_resource<ResourceContainer>(container_name) };
        if (p_container) {
            return p_container->create_resource<T>(new_res, active);
        }
        return nullptr;
    }

    template <class T>
    FLASHMEM typename std::enable_if<std::is_base_of<ResourceContainer, T>::value, T*>::type create_resource(const std::string_view& name, bool active) {
        std::unique_ptr<ResourceBase> p_res { std::make_unique<T>() };
        T* ptr { static_cast<T*>(p_res.get()) };
        if (add_resource(name, std::move(p_res), active)) {
            return ptr;
        }
        return nullptr;
    }

    FLASHMEM bool add_resource(const std::string_view& name, ResourceBase::Ptr&& p_res, bool active);

    ResourceBase* get_resource(const std::string_view& name) const {
        if (name.find(".") != name.npos) {
            return get_resource_deep(name);
        }
        if (!get_id(name)) {
            return nullptr;
        }
        return std::get<1>(resources_.find(name)->second).get();
    }

    template <typename T>
    typename std::enable_if<!std::is_same<T, ResourceContainer>::value, Resource<T>*>::type get_resource(const std::string_view& name) const {
        Resource<T>* ptr {};
        get_resource(name, ptr);
        return ptr;
    }

    template <typename T>
    typename std::enable_if<std::is_same<T, ResourceContainer>::value, ResourceContainer*>::type get_resource(const std::string_view& name) const {
        ResourceContainer* ptr {};
        get_resource(name, ptr);
        return ptr;
    }

    template <typename T>
    bool get_resource(const std::string_view& name, T*& p_res) const {
        if (name.find(".") != name.npos) {
            p_res = static_cast<T*>(get_resource_deep(name));
        } else if (get_id(name)) {
            p_res = static_cast<T*>(std::get<1>(resources_.find(name)->second).get()); // FIXME: optimize
        } else {
            p_res = nullptr;
        }
        return p_res != nullptr;
    }

    template <typename T>
    T* get_res_ptr(const std::string_view& name) const {
        auto ptr { get_resource<T>(name) };
        if (!ptr) {
            return nullptr;
        }
        return &ptr->get_ref();
    }

    const auto& get_resources() const noexcept {
        return resources_;
    }

    FLASHMEM bool set_update_state(const std::string_view& name);

    void set_active(const std::string_view& name, bool active);

    void reset_update_states();

    FLASHMEM void register_listener(std::function<void(const ResourceContainer&)> func, void* owner);

    FLASHMEM void register_listener(std::function<void(const ResourceContainer&)> func);

    FLASHMEM bool remove_listener(void* owner);
};

} // namespace ctbot

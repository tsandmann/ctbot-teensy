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
 * @file    resource_container.cpp
 * @brief   Aggregation for resources
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "resource_container.h"

#include <algorithm>


namespace ctbot {

ResourceContainer::ResourceContainer() noexcept : next_id_ { 1 } {}

ResourceContainer::~ResourceContainer() = default;

bool ResourceContainer::add_resource(const std::string_view& name, ResourceBase::Ptr&& p_res, bool active) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (resources_.find(name) != resources_.end()) {
        return false;
    }
    resources_[std::string { name }] = std::make_tuple(next_id_, std::move(p_res));
    if (active) {
        resources_active_.push_back(next_id_);
    }
    ++next_id_;
    return true;
}

ResourceBase* ResourceContainer::get_resource_deep(const std::string_view& name) const {
    const size_t local_part { name.find(".") };
    if (local_part == name.npos) {
        return nullptr;
    }
    const auto local_name { name.substr(0, local_part + 1) };
    const auto& res { resources_.find(local_name)->second };
    ResourceContainer* p_next { static_cast<ResourceContainer*>(std::get<1>(res).get()) };
    const auto nested_name { name.substr(local_part + 1) };
    if (nested_name.length()) {
        ResourceBase* ptr { nullptr };
        p_next->get_resource(nested_name, ptr);
        return ptr;
    } else {
        return p_next;
    }
}

bool ResourceContainer::set_update_state(const std::string_view& name) {
    std::unique_lock<std::mutex> lock(mutex_);

    const uint32_t id { get_id(name) };
    if (!id) {
        return false;
    }
    resource_updates_[id] = true;
    if (resource_updates_.size() == resources_active_.size()) {
        lock.unlock();
        call_listener();
        return true;
    }
    return false;
}

void ResourceContainer::set_active(const std::string_view& name, bool active) {
    std::lock_guard<std::mutex> lock(mutex_);

    const uint32_t id { get_id(name) };
    if (!id) {
        return;
    }
    if (active) {
        if (std::find(resources_active_.begin(), resources_active_.end(), id) == resources_active_.end()) {
            resources_active_.push_back(id);
        }
    } else {
        auto it = std::find(resources_active_.begin(), resources_active_.end(), id);
        if (it != resources_active_.end()) {
            resources_active_.erase(it);
        }
    }
}

void ResourceContainer::reset_update_states() {
    std::lock_guard<std::mutex> lock(mutex_);

    resource_updates_.clear();
}

void ResourceContainer::register_listener(std::function<void(const ResourceContainer&)> func, void* owner) {
    std::lock_guard<std::mutex> lock(mutex_);

    listeners_.push_back(std::make_tuple(func, owner));
}

void ResourceContainer::register_listener(std::function<void(const ResourceContainer&)> func) {
    register_listener(func, nullptr);
}

bool ResourceContainer::remove_listener(void* owner) {
    std::lock_guard<std::mutex> lock(mutex_);

    bool ret { false };
    for (auto it { listeners_.begin() }; it != listeners_.end();) {
        if (std::get<1>(*it) == owner) {
            listeners_.erase(it);
            ret = true;
        } else {
            ++it;
        }
    }

    return ret;
}

void ResourceContainer::call_listener() const {
    for (auto& func : listeners_) {
        std::get<0>(func)(*this);
    }
}

} // namespace ctbot

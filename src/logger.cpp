/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    logger.cpp
 * @brief   Logger for ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    14.12.2021
 */

#include "logger.h"

#include "ctbot.h"
#include "ctbot_cli.h"
#include "fs_service.h"

#include <cstdio>
#include <cstdarg>


namespace ctbot {

FLASHMEM LoggerTarget::LoggerTarget() {}

FLASHMEM LoggerTarget::~LoggerTarget() {}

int LoggerTarget::get_format_size(const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    const auto size { std::vsnprintf(nullptr, 0, format, vl) };
    va_end(vl);
    return size;
}

std::expected<std::string, int> LoggerTarget::create_formatted_string(size_t size, const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    auto str { std::string(size + 1, '\0') };
    const auto n { std::vsnprintf(str.data(), size + 1, format, vl) };
    if (n < 0) {
        return std::unexpected { n };
    }

    str.resize(n);
    va_end(vl);

    return str;
}


FLASHMEM LoggerTargetFile::LoggerTargetFile(FS_Service& fs_svc, const std::string_view& filename, CtBot* p_ctbot)
    : p_file_ {}, p_file_wrapper_ {}, p_ctbot_ { p_ctbot } {
    p_file_ = new File { fs_svc.open(std::string(filename).c_str(), static_cast<uint8_t>(FILE_WRITE), 0, &p_file_wrapper_) };
    configASSERT(p_file_ && p_file_wrapper_);
}

FLASHMEM LoggerTargetFile::~LoggerTargetFile() {
    p_file_->close();
    delete p_file_;
}

void LoggerTargetFile::begin(const std::string_view& prefix) {
    if (!*p_file_) {
        return;
    }

    auto p_str { new std::string };

    p_ctbot_->get_cli()->format_time(*p_str);
    p_str->append(PSTR(": "), 2);

    if (prefix.size()) {
        p_str->append(PSTR("["), 1);
        p_str->append(prefix.cbegin(), prefix.size());
        p_str->append(PSTR("] "), 2);
    }
    p_file_wrapper_->write(p_str->data(), p_str->size(), [p_str, this](FS_Service::FileOperation*) { delete p_str; });
}

size_t LoggerTargetFile::log(char c, bool) {
    if (!*p_file_) {
        return 0;
    }

    auto ptr { new char { c } };
    const auto res { p_file_wrapper_->write(ptr, 1, [ptr, this](FS_Service::FileOperation*) {
        delete ptr;
        p_file_wrapper_->flush([](FS_Service::FileOperation*) {});
    }) };

    return res;
}

size_t LoggerTargetFile::log(const std::string_view& str, bool) {
    if (!*p_file_) {
        return 0;
    }

    auto p_str { new std::string { str } };
    const auto res { p_file_wrapper_->write(p_str->data(), p_str->size(), [p_str, this](FS_Service::FileOperation*) {
        delete p_str;
        p_file_wrapper_->flush([](FS_Service::FileOperation*) {});
    }) };

    return res;
}

void LoggerTargetFile::flush() {
    p_file_->flush();
}


FLASHMEM Logger::Logger() {}

FLASHMEM size_t Logger::add_target(LoggerTarget* p_target) {
    targets_.push_back(p_target);
    return targets_.size() - 1;
}

void Logger::begin(const std::string_view& prefix) const {
    for (auto t : targets_) {
        t->begin(prefix);
    }
}

size_t Logger::log(char c, bool block) {
    size_t ret {};
    for (auto t : targets_) {
        const auto tmp { t->log(c, block) };
        if (tmp > ret) {
            ret = tmp;
        }
    }

    return ret;
}

size_t Logger::log(const char* str, bool block) {
    return log(std::string_view { str }, block);
}

size_t Logger::log(const std::string* p_str, bool block) {
    return log(*p_str, block);
}

size_t Logger::log(const std::string& str, bool block) {
    return log(std::string_view { str }, block);
}

size_t Logger::log(std::string&& str, bool block) {
    return log(std::string_view { str }, block);
}

size_t Logger::log(const std::string_view& str, bool block) {
    size_t ret {};
    for (auto t : targets_) {
        const auto tmp { t->log(str, block) };
        if (tmp > ret) {
            ret = tmp;
        }
    }

    return ret;
}

void Logger::flush() {
    for (auto t : targets_) {
        t->flush();
    }
}

} // namespace ctbot

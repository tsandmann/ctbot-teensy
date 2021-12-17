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

#ifdef ArduinoFiles_h
#undef FILE_READ
#endif
#ifdef ArduinoFiles_h
#undef FILE_WRITE
#endif
#include "FS.h"

#include <cstdarg>
#include <cstdio>


namespace ctbot {

FLASHMEM LoggerTarget::LoggerTarget() {}

FLASHMEM LoggerTarget::~LoggerTarget() {}


FLASHMEM LoggerTargetFile::LoggerTargetFile(FS& fs, const std::string_view& filename, CtBot* p_ctbot) : p_file_ {}, p_ctbot_ { p_ctbot } {
    p_file_ = new File { fs.open(std::string(filename).c_str(), static_cast<uint8_t>(FILE_WRITE)) };
    configASSERT(p_file_);
}

FLASHMEM LoggerTargetFile::~LoggerTargetFile() {
    p_file_->close();
    delete p_file_;
}

void LoggerTargetFile::begin(const std::string_view& prefix) const {
    if (!*p_file_) {
        return;
    }

    std::string str;
    p_ctbot_->get_cli()->format_time(str);
    p_file_->write(str.c_str());
    p_file_->write(PSTR(": "));
    if (prefix.size()) {
        p_file_->write(PSTR("["));
        p_file_->write(prefix.cbegin(), prefix.size());
        p_file_->write(PSTR("] "));
    }
}

size_t LoggerTargetFile::log(const char c, const bool) {
    if (!*p_file_) {
        return 0;
    }

    return p_file_->write(c);
}

size_t LoggerTargetFile::log(const std::string_view& str, const bool) {
    if (!*p_file_) {
        return 0;
    }

    return p_file_->write(str.cbegin(), str.size());
}

void LoggerTargetFile::flush() {
    p_file_->flush();
}


FLASHMEM Logger::Logger() {}

size_t Logger::get_format_size(const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    const auto size { std::vsnprintf(nullptr, 0, format, vl) + 1 };
    va_end(vl);
    return size;
}

std::string Logger::create_formatted_string(const size_t size, const char* format, ...) {
    va_list vl;
    va_start(vl, format);
    auto str { std::string(size + 32, '\0') };
    std::vsnprintf(str.data(), size, format, vl);
    va_end(vl);

    return str;
}

FLASHMEM size_t Logger::add_target(LoggerTarget* p_target) {
    targets_.push_back(p_target);
    return targets_.size() - 1;
}

void Logger::begin(const std::string_view& prefix) const {
    for (auto t : targets_) {
        t->begin(prefix);
    }
}

size_t Logger::log(const char c, const bool block) {
    size_t ret {};
    for (auto t : targets_) {
        const auto tmp { t->log(c, block) };
        if (tmp > ret) {
            ret = tmp;
        }
    }

    return ret;
}

size_t Logger::log(const char* str, const bool block) {
    return log(std::string_view { str }, block);
}

size_t Logger::log(const std::string* p_str, const bool block) {
    return log(*p_str, block);
}

size_t Logger::log(const std::string& str, const bool block) {
    return log(std::string_view { str }, block);
}

size_t Logger::log(std::string&& str, const bool block) {
    return log(std::string_view { str }, block);
}

size_t Logger::log(const std::string_view& str, const bool block) {
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

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
 * @file    cmd_parser.h
 * @brief   Command interface parser
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "cmd_parser.h"
#include "ctbot.h"

#include "pprintpp.hpp"

#include <cstdlib>


namespace ctbot {

CmdParser::CmdParser() : echo_ {}, p_fs_svc_ {}, p_file_ {}, p_file_wrapper_ {} {}

CmdParser::~CmdParser() {
    if (p_file_) {
        p_file_->close();
        delete p_file_;
    }
}

void CmdParser::register_cmd(const std::string_view& cmd, func_t&& func) {
    commands_.emplace(std::make_pair(cmd, func));

    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("CmdParser::register_cmd(\"%s\"): cmd=0x%x addr=0x%x size=%u\r\n"), cmd.cbegin(), cmd.data(),
            commands_.find(cmd)->first.data(), commands_.find(cmd)->first.size());
    }
}

void CmdParser::register_cmd(const std::string_view& cmd, const char* cmd_short, func_t&& func) {
    commands_.emplace(std::make_pair(std::string_view { cmd_short, 1 }, func));

    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf<true>(
            PSTR("CmdParser::register_cmd('%s'): addr=0x%x\r\n"), cmd_short, commands_.find(std::string_view { cmd_short, 1 })->first.data());
    }

    register_cmd(cmd, std::move(func));
}

bool CmdParser::execute_cmd(const std::string_view& cmd, CommInterface& comm) {
    if (!cmd.size()) {
        return false;
    }

    if (cmd[0] == '#') {
        // just a comment
        return true;
    }

    const auto arg_pos { cmd.find(" ") };
    const auto cmd_str { cmd.substr(0, arg_pos) };

    if constexpr (DEBUG_) {
        comm.debug_printf<true>(PSTR("arg_pos=%u cmd_str=\"%s\"\r\n"), arg_pos, std::string { cmd_str }.c_str());
    }

    const auto it { commands_.find(cmd_str) };
    if (it == commands_.end()) {
        if (echo_) {
            comm.set_color(CommInterface::Color::RED, CommInterface::Color::BLACK);
            comm.set_attribute(CommInterface::Attribute::BOLD);
            comm.debug_print(PSTR("ERROR"), true);
            comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
            comm.debug_print(PSTR(": command \""), true);
            comm.debug_print(cmd, true);
            comm.debug_print(PSTR("\" not found.\r\n"), true);
            comm.set_attribute(CommInterface::Attribute::NORMAL);
            if constexpr (DEBUG_) {
                comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
                comm.debug_printf<true>(PP_ARGS("cmd.size={}\r\n", cmd.size()));
                for (size_t i {}; i < cmd.size(); ++i) {
                    comm.debug_printf<true>(PP_ARGS("{#x} ", static_cast<uint16_t>(cmd[i])));
                }
            }
            comm.debug_print(PSTR("\r\n"), true);
        }
        return false;
    }

    std::string_view args;
    if (arg_pos != cmd.npos && cmd.size() > arg_pos + 1) {
        args = cmd.substr(arg_pos + 1);
    }

    const bool res { it->second(args) };

    if (res && echo_) {
        comm.set_color(CommInterface::Color::GREEN, CommInterface::Color::BLACK);
        comm.set_attribute(CommInterface::Attribute::BOLD);
        comm.debug_print("OK\r\n", true);
        comm.set_attribute(CommInterface::Attribute::NORMAL);
        comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
    } else if (echo_) {
        comm.set_color(CommInterface::Color::RED, CommInterface::Color::BLACK);
        comm.debug_print("ERROR\r\n", true);
        comm.set_attribute(CommInterface::Attribute::NORMAL);
        comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
    }

    return res;
}

bool CmdParser::parse(const std::string_view& in, CommInterface& comm) {
    if (!in.empty()) {
        history_.emplace_front(in);
        if (history_.size() > HISTORY_SIZE_) {
            history_.pop_back();
        }
        const auto result { execute_cmd(history_.front(), comm) };

        if constexpr (CtBotConfig::SDCARD_AVAILABLE && CtBotConfig::CLI_HISTORY_ON_SDCARD_AVAILABLE) {
            if (result && p_file_wrapper_) {
                auto p_str { new std::string { history_.front() } };
                p_str->append(PSTR("\r\n"));
                p_file_wrapper_->write(p_str->data(), p_str->size(), [p_str, this](FS_Service::FileOperation*) {
                    delete p_str;
                    p_file_wrapper_->flush([](FS_Service::FileOperation*) {});
                });
            }
        }

        return result;
    }

    return false;
}

bool CmdParser::enable_nv_history(FS_Service& fs_svc, const std::string_view& filename) {
    if constexpr (CtBotConfig::SDCARD_AVAILABLE && CtBotConfig::CLI_HISTORY_ON_SDCARD_AVAILABLE) {
        p_fs_svc_ = &fs_svc;
        p_file_ = new File { p_fs_svc_->open(std::string(filename).c_str(), static_cast<uint8_t>(FILE_WRITE), 0, &p_file_wrapper_) };

        if (!p_file_wrapper_ && p_file_) {
            delete p_file_;
            p_file_ = nullptr;
        }

        return p_file_wrapper_ != nullptr;
    }

    return false;
}

std::optional<std::string> CmdParser::get_history(size_t num) {
    if (num && num <= history_.size()) {
        return history_[num - 1];
    } else if (CtBotConfig::SDCARD_AVAILABLE && CtBotConfig::CLI_HISTORY_ON_SDCARD_AVAILABLE && num && p_file_wrapper_) {
        configASSERT(p_fs_svc_);
        auto file { p_fs_svc_->open(p_file_wrapper_->name(), FILE_READ, 5'000) };
        if (file) {
            const auto size { static_cast<int32_t>(file.size()) };
            if (size < 0) {
                return {};
            }

            uint32_t pos_end;
            int32_t pos { 1 };
            for (size_t i {}; i <= num; ++i) {
                pos_end = static_cast<uint32_t>(file.position());
                for (; pos <= size; ++pos) {
                    file.seek(-pos, SeekEnd);
                    const auto tmp { file.read() };
                    if (tmp == '\n') {
                        ++pos;
                        break;
                    }
                }
            }
            const auto pos_start { static_cast<uint32_t>(file.position()) + 1 };
            if (pos <= size) {
                const size_t len { pos_end - pos_start - 1 };
                std::string line(static_cast<size_t>(len), '\0');
                if (file.readBytes(line.data(), len) == len) {
                    return line;
                }
            }
        }
    }
    return {};
}

std::string_view CmdParser::trim_to_first_arg(const std::string_view& str) {
    if (!str.size()) {
        return str;
    }
    const auto start { str.find_first_of(' ') };
    if (start == str.npos) {
        return str;
    }

    auto tmp { str };
    tmp.remove_prefix(std::min(tmp.find_first_not_of(' ', start), tmp.size()));

    return tmp;
}

} // namespace ctbot

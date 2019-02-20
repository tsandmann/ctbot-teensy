
/**
 * @file    sim_connection.cpp
 * @brief   C't-Sim connection
 * @author  Timo Sandmann
 * @date    17.06.2018
 */

#include "sim_connection.h"
#include "command.h"
#include "../../../src/ctbot.h"
#include "../../../src/ctbot_config.h"
#include "../../../src/motor.h"
#include "../../../src/servo.h"
#include "../../../src/leds.h"
#include "../../../src/sensors.h"
#include "../../../src/encoder.h"

#include "FreeRTOS.h"
#include "task.h"
#include "arduino_fixed.h"
#include <iostream>
#include <boost/system/system_error.hpp>


namespace arduino {
static uint32_t g_time_ms { 0 };

uint32_t micros() {
    return g_time_ms * 1000UL;
}

uint32_t millis() {
    return g_time_ms;
}
} // namespace arduino

namespace ctbot {

SimConnection::SimConnection(const std::string& hostname, const std::string& port)
    : sock_ { io_service_ }, bot_addr_ { CommandBase::ADDR_BROADCAST }, sim_time_ms_ { -11 } {
    boost::asio::ip::tcp::resolver r { io_service_ };
    boost::asio::ip::tcp::resolver::query q { boost::asio::ip::tcp::v4(), hostname, port };
    boost::asio::ip::tcp::resolver::iterator it { r.resolve(q) };
    try {
        sock_.connect(*it);
        std::cout << "SimConnection::SimConnection(): connected to ct-Sim.\n";
    } catch (const boost::system::system_error& e) {
        std::cerr << "SimConnection::SimConnection(): connect to ct-Sim failed: \"" << e.what() << "\"\n";
        sock_.close();
        return;
    }

    register_cmd(CommandCodes::CMD_WELCOME, [this](const CommandBase& cmd) {
        // std::cout << "CMD_WELCOME received: " << cmd << "\n";
        CommandNoCRC cmd1 { CommandCodes::CMD_ID, CommandCodes::CMD_SUB_ID_REQUEST, 0, 0, CommandBase::ADDR_BROADCAST };
        send_cmd(cmd1);
        return true;
    });
    register_cmd(CommandCodes::CMD_DONE, [this](const CommandBase& cmd) {
        int16_t last_time { sim_time_ms_ };
        sim_time_ms_ = cmd.get_cmd_data_l();
        int32_t diff { sim_time_ms_ - last_time };
        if (diff < 0) {
            diff += 10000;
        }
        arduino::g_time_ms += diff;
        // std::cout << "CMD_DONE received: " << cmd << "sim_time _ms_=" << std::dec << sim_time_ms_ << " g_time_ms=" << arduino::g_time_ms << "\n";
        return true;
    });
    register_cmd(CommandCodes::CMD_ID, [this](const CommandBase& cmd) {
        if (cmd.get_cmd_subcode() == CommandCodes::CMD_SUB_ID_OFFER) {
            bot_addr_ = cmd.get_cmd_data_l();
            // std::cout << "CMD_ID received: " << cmd << std::dec << "bot_addr_=" << static_cast<uint16_t>(bot_addr_) << "\n";
            CommandNoCRC cmd1 { CommandCodes::CMD_ID, CommandCodes::CMD_SUB_ID_SET, bot_addr_, 0, bot_addr_ };
            send_cmd(cmd1);
        }
        return true;
    });
    register_cmd(CommandCodes::CMD_SHUTDOWN, [](const CommandBase& cmd) {
        // std::cout << "CMD_SHUTDOWN received: " << cmd << "\n";
        CtBot::get_instance().stop();
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_IR, [](const CommandBase& cmd) {
        arduino::analogWrite(CtBotConfig::DISTANCE_L_PIN, cmd.get_cmd_data_l());
        arduino::analogWrite(CtBotConfig::DISTANCE_R_PIN, cmd.get_cmd_data_r());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_ENC, [](const CommandBase& cmd) {
        // std::cout << "CMD_SENS_ENC received: " << cmd << "\n";

        const int16_t diff_l { cmd.get_cmd_data_l() };
        if (diff_l) {
            // std::cout << "CMD_SENS_ENC received: diff_l=" << diff_l << "\n";

            uint8_t idx { DigitalSensors::enc_l_idx_ };
            ++idx;
            idx %= Encoder::DATA_ARRAY_SIZE;
            DigitalSensors::enc_l_idx_ = idx;
            DigitalSensors::enc_data_l_[idx] = Timer::get_us();
        }

        const int16_t diff_r { cmd.get_cmd_data_r() };
        if (diff_r) {
            // std::cout << "CMD_SENS_ENC received: diff_r=" << diff_r << "\n";

            uint8_t idx { DigitalSensors::enc_r_idx_ };
            ++idx;
            idx %= Encoder::DATA_ARRAY_SIZE;
            DigitalSensors::enc_r_idx_ = idx;
            DigitalSensors::enc_data_r_[idx] = Timer::get_us();
        }

        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_BORDER, [](const CommandBase& cmd) {
        arduino::analogWrite(CtBotConfig::BORDER_L_PIN, cmd.get_cmd_data_l());
        arduino::analogWrite(CtBotConfig::BORDER_R_PIN, cmd.get_cmd_data_r());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_LINE, [](const CommandBase& cmd) {
        arduino::analogWrite(CtBotConfig::LINE_L_PIN, cmd.get_cmd_data_l());
        arduino::analogWrite(CtBotConfig::LINE_R_PIN, cmd.get_cmd_data_r());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_LDR, [](const CommandBase& cmd) {
        arduino::analogWrite(CtBotConfig::LDR_L_PIN, cmd.get_cmd_data_l());
        arduino::analogWrite(CtBotConfig::LDR_R_PIN, cmd.get_cmd_data_r());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_TRANS, [](const CommandBase& cmd) {
        arduino::digitalWriteFast(CtBotConfig::TRANSPORT_PIN, cmd.get_cmd_data_l());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_DOOR, [](const CommandBase& cmd) {
        arduino::digitalWriteFast(CtBotConfig::SHUTTER_PIN, cmd.get_cmd_data_l());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_RC5, [](const CommandBase& cmd) {
        const int16_t data { cmd.get_cmd_data_l() };
        const uint8_t rc5_addr { static_cast<uint8_t>((data & 0x7c0) >> 6) };
        const uint8_t rc5_cmd { static_cast<uint8_t>(data & 0x3f) };
        auto p_sensors { CtBot::get_instance().get_sensors() };
        if (p_sensors) {
            p_sensors->get_rc5().set_rc5(rc5_addr, rc5_cmd);
        }
        // if (data) {
        //     std::cout << "CMD_SENS_RC5 received: data=0x" << std::hex << data << " addr=0x" << static_cast<uint16_t>(rc5_addr)
        //         << " cmd=0x" << static_cast<uint16_t>(rc5_cmd) << std::dec << "\n";
        // }
        return true;
    });

    register_cmd(CommandCodes::CMD_SENS_ERROR, [](const CommandBase&) { return true; });
    register_cmd(CommandCodes::CMD_SENS_MOUSE, [](const CommandBase&) { return true; });
    register_cmd(CommandCodes::CMD_SENS_BPS, [](const CommandBase&) { return true; });
    register_cmd(CommandCodes::CMD_AKT_SERVO, [](const CommandBase&) { return true; });

    {
        CommandNoCRC cmd { CommandCodes::CMD_WELCOME, CommandCodes::CMD_SUB_WELCOME_SIM, 2 /* RC5 */, 0, CommandBase::ADDR_BROADCAST };
        if (!send_cmd(cmd)) {
            std::cerr << "SimConnection::SimConnection(): send_cmd() failed.\n";
        }
    }
    if (receive_sensor_data()) {
        sim_time_ms_ += 10;
        CommandNoCRC cmd { CommandCodes::CMD_DONE, CommandCodes::CMD_SUB_NORM, sim_time_ms_, 0, CommandBase::ADDR_BROADCAST };
        if (!send_cmd(cmd)) {
            std::cerr << "SimConnection::SimConnection(): send_cmd() failed.\n";
        }
    }

    p_recv_thread_ = std::make_shared<std::thread>([this]() {
        while (sock_.is_open()) {
            if (CtBot::get_instance().get_ready() && receive_sensor_data()) {
                {
                    const auto mot_l_pwm { arduino::analogRead(CtBotConfig::MOT_L_PWM_PIN)
                        * (arduino::digitalReadFast(CtBotConfig::MOT_L_DIR_PIN) ? 450 / 2 : -450 / 2) / (1 << Motor::PWM_RESOLUTION) };
                    const auto mot_r_pwm { arduino::analogRead(CtBotConfig::MOT_R_PWM_PIN)
                        * (arduino::digitalReadFast(CtBotConfig::MOT_R_DIR_PIN) ? -450 / 2 : 450 / 2) / (1 << Motor::PWM_RESOLUTION) };
                    CommandNoCRC cmd { CommandCodes::CMD_AKT_MOT, CommandCodes::CMD_SUB_NORM, static_cast<int16_t>(mot_l_pwm), static_cast<int16_t>(mot_r_pwm),
                        bot_addr_ };
                    send_cmd(cmd);
                }

                {
                    auto p_servo_1 { CtBot::get_instance().get_servos()[0] };
                    auto p_servo_2 { CtBot::get_instance().get_servos()[1] };
                    CommandNoCRC cmd { CommandCodes::CMD_AKT_SERVO, CommandCodes::CMD_SUB_NORM, static_cast<int16_t>(p_servo_1 ? p_servo_1->get_position() : 0),
                        static_cast<int16_t>(p_servo_2 ? p_servo_2->get_position() : 0), bot_addr_ };
                    send_cmd(cmd);
                }

                {
                    auto p_leds { CtBot::get_instance().get_leds() };
                    CommandNoCRC cmd { CommandCodes::CMD_AKT_LED, CommandCodes::CMD_SUB_NORM,
                        static_cast<int16_t>(p_leds ? static_cast<int16_t>(p_leds->get()) : 0), 0, bot_addr_ };
                    send_cmd(cmd);
                }

                {
                    CommandNoCRC cmd { CommandCodes::CMD_DONE, CommandCodes::CMD_SUB_NORM, sim_time_ms_, 0, bot_addr_ };
                    send_cmd(cmd);
                }
            }
        }
    });
}

SimConnection::~SimConnection() {
    sock_.close();
    if (p_recv_thread_ && p_recv_thread_->joinable()) {
        p_recv_thread_->join();
    }
}

size_t SimConnection::receive_until(std::streambuf& buf, const char delim, const size_t maxsize) {
    if (!sock_.is_open()) {
        std::cerr << "SimConnection::receive_until(): not ready, abort.\n";
        return 0;
    }

    boost::system::error_code error;
    auto& buffer { dynamic_cast<boost::asio::streambuf&>(buf) };
    const auto n { std::min(boost::asio::read_until(sock_, buffer, delim, error), maxsize) };

    if (error) {
        throw boost::system::system_error(error);
    }

    return n;
}

size_t SimConnection::send(const void* data, const size_t size) {
    if (!sock_.is_open()) {
        std::cerr << "SimConnection::send(): not ready, abort.\n";
        return 0;
    }

    boost::system::error_code error;
    const auto n { boost::asio::write(sock_, boost::asio::buffer(data, size), error) };

    if (error) {
        throw boost::system::system_error(error);
    }

    return n;
}

bool SimConnection::receive_sensor_data() {
    return receive_sensor_data(CommandCodes::CMD_DONE);
}

bool SimConnection::receive_sensor_data(const CommandCodes cmd) {
    if (!sock_.is_open()) {
        std::cerr << "SimConnection::receive_sensor_data(): not ready, abort.\n";
        return false;
    }

    std::shared_ptr<CommandBase> p_cmd;
    do {
        try {
            if (!receive_until(recv_bufer_, static_cast<const char>(CommandCodes::CMD_STOPCODE), 1024)) {
                return false;
            }
        } catch (const boost::system::system_error& e) {
            if (e.code() != boost::asio::error::basic_errors::interrupted) {
                std::cerr << "SimConnection::receive_sensor_data(): receiving commands failed: \"" << e.what() << "\"\n";
            }
            return false;
        }

        try {
            p_cmd = std::make_shared<CommandNoCRC>(recv_bufer_);
        } catch (const std::exception& e) {
            std::cerr << "SimConnection::receive_sensor_data(): creating new command failed: \"" << e.what() << "\"\n";
            return false;
        }
        assert(p_cmd);
        if (p_cmd->get_payload_size()) {
            p_cmd->append_payload(recv_bufer_, p_cmd->get_payload_size());
        }
        if (!evaluate_cmd(*p_cmd)) {
            return false;
        }
    } while (p_cmd->get_cmd_code() != cmd);

    return true;
}

bool SimConnection::send_cmd(CommandBase& cmd) {
    if (!sock_.is_open()) {
        return false;
    }

    if (cmd.get_cmd_from() == CommandBase::ADDR_NOT_SET) {
        cmd.set_cmd_from(bot_addr_);
    }
    if (cmd.get_cmd_to() == CommandBase::ADDR_NOT_SET) {
        cmd.set_cmd_to(CommandBase::ADDR_SIM);
    }

    try {
        auto sent { send(&cmd.get_cmd(), sizeof(CommandData)) };
        if (cmd.get_payload_size()) {
            sent += send(cmd.get_payload().data(), cmd.get_payload_size());
        }
        return sent == sizeof(CommandData) + cmd.get_payload_size();
    } catch (const boost::system::system_error& e) {
        return false;
    }
}

bool SimConnection::send_cmd(std::vector<std::shared_ptr<CommandBase>>& cmds) {
    if (!sock_.is_open()) {
        return false;
    }

    try {
        for (auto p_cmd : cmds) {
            if (p_cmd->get_cmd_from() == CommandBase::ADDR_NOT_SET) {
                p_cmd->set_cmd_from(bot_addr_);
            }
            if (p_cmd->get_cmd_to() == CommandBase::ADDR_NOT_SET) {
                p_cmd->set_cmd_to(CommandBase::ADDR_SIM);
            }
            auto sent { send(&p_cmd->get_cmd(), sizeof(CommandData)) };
            if (p_cmd->get_payload_size()) {
                sent += send(p_cmd->get_payload().data(), p_cmd->get_payload_size());
            }
            if (sent != sizeof(CommandData) + p_cmd->get_payload_size()) {
                return false;
            }
        }
    } catch (const boost::system::system_error& e) {
        std::cerr << "SimConnection::send_cmd(): send command failed: \"" << e.what() << "\"\n";
        return false;
    }
    return true;
}

bool SimConnection::evaluate_cmd(const CommandBase& cmd) {
    const CommandData& cmd_data(cmd.get_cmd());
    if (!cmd.get_cmd_has_crc() && cmd_data.from != CommandBase::ADDR_SIM) {
        return false;
    }

    try {
        bool result { true };
        for (auto& func : commands_.at(cmd.get_cmd_code())) {
            result &= func(cmd);
        }
        return result;
    } catch (const std::out_of_range&) {
        std::cerr << "SimConnection::evaluate_cmd(): CMD code '" << static_cast<char>(cmd.get_cmd_code_uint()) << "' not registered.\n";
        // std::cout << cmd;
        return false;
    }
}

void SimConnection::register_cmd(const CommandCodes& cmd, decltype(commands_)::mapped_type::value_type func) {
    commands_[cmd].push_back(func);
}

} /* namespace ctbot */

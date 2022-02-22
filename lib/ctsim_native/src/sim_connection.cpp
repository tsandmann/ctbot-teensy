
/**
 * @file    sim_connection.cpp
 * @brief   ct-Sim connection
 * @author  Timo Sandmann
 * @date    17.06.2018
 */

#include "sim_connection.h"
#include "command.h"
#include "../../../src/ctbot.h"
#include "../../../src/ctbot_config.h"
#include "../../../src/i2c_service.h"
#include "../../../src/sensors.h"
#include "../../../src/speed_control.h"
#include "../../../src/behavior/ctbot_behavior.h"
#include "../../../src/driver/encoder.h"
#include "../../../src/driver/lc_display.h"
#include "../../../src/driver/leds_i2c.h"
#include "../../../src/driver/motor.h"
#include "../../../src/driver/servo.h"
#include "../../../src/driver/vl53l0x.h"
#include "../../../src/driver/vl6180x.h"

#include "circular_buffer.h"
#include "arduino_freertos.h"
#include "LiquidCrystal_I2C.h"

#include <boost/bind/bind.hpp>
#include <boost/system/system_error.hpp>
#include <chrono>
#include <iostream>


namespace arduino {
static uint32_t g_time_ms {};

uint32_t micros() {
    return g_time_ms * 1'000UL;
}

uint32_t millis() {
    return g_time_ms;
}
} // namespace arduino


namespace ctbot {
uint32_t Timer::get_us() {
    return arduino::g_time_ms * 1'000UL;
}

uint32_t Timer::get_us_from_isr() {
    return arduino::g_time_ms * 1'000UL;
}

uint32_t Timer::get_ms() {
    return arduino::g_time_ms;
}

SimConnection::SimConnection(const std::string& hostname, const std::string& port)
    : sock_ { io_service_ }, bot_addr_ { CommandBase::ADDR_BROADCAST }, sim_time_ms_ { -1 }, p_i2c_range_ {} {
    boost::asio::ip::tcp::resolver r { io_service_ };
    boost::asio::ip::tcp::resolver::query q { boost::asio::ip::tcp::v4(), hostname, port };
    endpoint_it_ = r.resolve(q);

    p_i2c_range_ =
        CtBotConfig::VL53L0X_I2C_BUS == 0 ? &Wire : (CtBotConfig::VL53L0X_I2C_BUS == 1 ? &Wire1 : (CtBotConfig::VL53L0X_I2C_BUS == 2 ? &Wire2 : &Wire3));

    p_i2c_range_->set_addr_width(CtBotConfig::VL6180X_I2C_ADDR, 2);
    i2c_write_reg8(p_i2c_range_, VL53L0X::DEFAULT_I2C_ADDR, VL53L0X::MODEL_ID_REG, 0xee);
    i2c_write_reg8(p_i2c_range_, VL53L0X::DEFAULT_I2C_ADDR, VL53L0X::RESULT_INTERRUPT_STATUS_REG, 7);
    i2c_write_reg8(p_i2c_range_, CtBotConfig::VL53L0X_L_I2C_ADDR, VL53L0X::MODEL_ID_REG, 0xee);
    i2c_write_reg8(p_i2c_range_, CtBotConfig::VL53L0X_R_I2C_ADDR, VL53L0X::MODEL_ID_REG, 0xee);

    p_io_thread_ = std::make_unique<std::thread>([this]() {
        using namespace std::chrono_literals;

        ::vTaskPrioritySet(nullptr, 1);
        while (!CtBot::get_instance().get_ready()) {
        }
        ::vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 1);

        auto p_model { CtBotBehavior::get_instance().get_data()->get_resource<ResourceContainer>("model.") };
        assert(p_model);
        p_sim_res_ = p_model->create_resource<bool>("sim_update", true);
        assert(p_sim_res_);
        p_sim_res_->register_listener([p_model](const Resource<bool>::basetype&) {
            p_model->set_update_state("sim_update");
            return;
        });

        sock_.async_connect(endpoint_it_->endpoint(), boost::bind(&SimConnection::handle_connect, this, boost::asio::placeholders::error));

        sigset_t set, old;
        sigfillset(&set);
        pthread_sigmask(SIG_SETMASK, &set, &old);

        while (CtBot::get_instance().get_ready()) {
            io_service_.poll();
            std::this_thread::sleep_for(1ms);
        }
        pthread_sigmask(SIG_SETMASK, &old, nullptr);
    });

    free_rtos_std::gthr_freertos::set_name(p_io_thread_.get(), "ct-Sim");
}

SimConnection::~SimConnection() {
    sock_.close();
    if (p_io_thread_ && p_io_thread_->joinable()) {
        p_io_thread_->join();
    }
}

void SimConnection::handle_connect(const boost::system::error_code& ec) {
    if (ec.value() == EINTR) {
        // std::cerr << "SimConnection::handle_connect(): connection request interrupted, trying again...\n";
        sock_.close();
        sock_.async_connect(endpoint_it_->endpoint(), boost::bind(&SimConnection::handle_connect, this, boost::asio::placeholders::error));
        return;
    }

    if (ec || !sock_.is_open()) {
        std::cerr << "SimConnection::handle_connect(): connection to ct-Sim failed: \"" << (ec ? ec.message() : "unknown error") << "\"\n";
        sock_.close();
        return;
    }

    boost::asio::ip::tcp::no_delay option { true };
    sock_.set_option(option);
    std::cout << "SimConnection::handle_connect(): connected to ct-Sim.\n";


    register_cmd(CommandCodes::CMD_WELCOME, [this](const CommandBase& cmd) {
        // std::cout << "CMD_WELCOME received: " << cmd << "\n";
        (void) cmd;
        CommandNoCRC cmd1 { CommandCodes::CMD_ID, CommandCodes::CMD_SUB_ID_REQUEST, 0, 0, CommandBase::ADDR_BROADCAST };
        send_cmd(cmd1);
        return true;
    });
    register_cmd(CommandCodes::CMD_DONE, [this](const CommandBase& cmd) {
        int16_t last_time { sim_time_ms_ };
        sim_time_ms_ = cmd.get_cmd_data_l();
        int32_t diff { sim_time_ms_ - last_time };
        if (diff < 0) {
            diff += 10'000;
        }
        arduino::g_time_ms += diff;
        // std::cout << "CMD_DONE received: " << cmd << "sim_time_ms_=" << std::dec << sim_time_ms_ << " g_time_ms=" << arduino::g_time_ms << "\n";


        if (CtBot::get_instance().get_ready()) {
            p_sim_res_->notify(); // FIXME: think about this

            {
                auto p_speedctrl_l { CtBot::get_instance().get_speedcontrols()[0] };
                auto p_speedctrl_r { CtBot::get_instance().get_speedcontrols()[1] };

                CommandNoCRC cmd { CommandCodes::CMD_AKT_MOT, CommandCodes::CMD_SUB_NORM,
                    static_cast<int16_t>(p_speedctrl_l->get_speed() * SpeedControlBase::MAX_SPEED / 200.f),
                    static_cast<int16_t>(p_speedctrl_r->get_speed() * SpeedControlBase::MAX_SPEED / 200.f), bot_addr_ };

                send_cmd(cmd);
            }

            {
                auto p_servo_1 { CtBot::get_instance().get_servos()[0] };
                auto p_servo_2 { CtBot::get_instance().get_servos()[1] };

                CommandNoCRC cmd { CommandCodes::CMD_AKT_SERVO, CommandCodes::CMD_SUB_NORM, static_cast<int16_t>(p_servo_1->get_position()),
                    static_cast<int16_t>(p_servo_2 ? p_servo_2->get_position() : 0), bot_addr_ };

                send_cmd(cmd);
            }

            {
                auto p_leds { CtBot::get_instance().get_leds() };

                CommandNoCRC cmd { CommandCodes::CMD_AKT_LED, CommandCodes::CMD_SUB_NORM,
                    static_cast<int16_t>(p_leds ? static_cast<int16_t>(p_leds->get()) : 0), 0, bot_addr_ };

                send_cmd(cmd);
            }

            if (CtBotConfig::LCD_AVAILABLE) {
                auto p_lcd { CtBot::get_instance().get_lcd()->get_impl() };
                int16_t r {};
                for (const auto& e : p_lcd->get_buffer()) {
                    CommandNoCRC cmd { CommandCodes::CMD_AKT_LCD, CommandCodes::CMD_SUB_LCD_DATA, 0, r++, bot_addr_ };
                    cmd.add_payload(e.c_str(), 20);
                    send_cmd(cmd);
                }
            }

            {
                CommandNoCRC cmd { CommandCodes::CMD_DONE, CommandCodes::CMD_SUB_NORM, sim_time_ms_, 0, bot_addr_ };
                send_cmd(cmd);
            }
        }

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
    register_cmd(CommandCodes::CMD_SHUTDOWN, [](const CommandBase&) {
        // std::cout << "CMD_SHUTDOWN received: " << cmd << "\n";
        CtBot::get_instance().stop();
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_IR, [this](const CommandBase& cmd) {
        // std::cout << "CMD_SENS_IR received: " << cmd << "\n";

        i2c_write_reg16(p_i2c_range_, CtBotConfig::VL53L0X_L_I2C_ADDR, static_cast<uint8_t>(VL53L0X::RESULT_RANGE_STATUS_REG + 10),
            static_cast<uint16_t>(cmd.get_cmd_data_l()));
        i2c_write_reg8(p_i2c_range_, CtBotConfig::VL53L0X_L_I2C_ADDR, VL53L0X::RESULT_INTERRUPT_STATUS_REG, 7);
        // FIXME: handle SYSTEM_INTERRUPT_CLEAR?

        i2c_write_reg16(p_i2c_range_, CtBotConfig::VL53L0X_R_I2C_ADDR, static_cast<uint8_t>(VL53L0X::RESULT_RANGE_STATUS_REG + 10),
            static_cast<uint16_t>(cmd.get_cmd_data_r()));
        i2c_write_reg8(p_i2c_range_, CtBotConfig::VL53L0X_R_I2C_ADDR, VL53L0X::RESULT_INTERRUPT_STATUS_REG, 7);

        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_ENC, [this](const CommandBase& cmd) {
        // std::cout << "CMD_SENS_ENC received: " << cmd << "\n";

        const int16_t diff_l { cmd.get_cmd_data_l() };
        // std::cout << "CMD_SENS_ENC received: diff_l=" << diff_l << "\n";

        for (int16_t i {}; i < std::abs(diff_l); ++i) {
            uint8_t idx { DigitalSensors::enc_l_idx_ };
            ++idx;
            idx %= Encoder::DATA_ARRAY_SIZE;
            DigitalSensors::enc_data_l_[idx] = arduino::g_time_ms * 1'000UL;
            DigitalSensors::enc_l_idx_ = idx;
        }

        const int16_t diff_r { cmd.get_cmd_data_r() };
        // std::cout << "CMD_SENS_ENC received: diff_r=" << diff_r << "\n";

        for (int16_t i {}; i < std::abs(diff_r); ++i) {
            uint8_t idx { DigitalSensors::enc_r_idx_ };
            ++idx;
            idx %= Encoder::DATA_ARRAY_SIZE;
            DigitalSensors::enc_data_r_[idx] = arduino::g_time_ms * 1'000UL;
            DigitalSensors::enc_r_idx_ = idx;
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
    register_cmd(CommandCodes::CMD_SENS_TRANS, [this](const CommandBase& cmd) {
        i2c_write_reg8(p_i2c_range_, CtBotConfig::VL6180X_I2C_ADDR, VL6180X::RESULT_RANGE_VAL_REG, cmd.get_cmd_data_l() > 0 ? 10 : 50);
        i2c_write_reg8(p_i2c_range_, CtBotConfig::VL6180X_I2C_ADDR, VL6180X::RESULT_INTERRUPT_STATUS_REG, 4);
        // FIXME: handle SYSTEM__INTERRUPT_CLEAR?
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_DOOR, [](const CommandBase&) {
        // arduino::digitalWriteFast(CtBotConfig::SHUTTER_PIN, cmd.get_cmd_data_l());
        return true;
    });
    register_cmd(CommandCodes::CMD_SENS_RC5, [](const CommandBase& cmd) {
        const int16_t data { cmd.get_cmd_data_l() };
        const uint8_t rc5_addr { static_cast<uint8_t>((data & 0x7c0) >> 6) };
        const uint8_t rc5_cmd { static_cast<uint8_t>(data & 0x3f) };
        const bool toggle { cmd.get_cmd_data_r() > 0 };
        auto p_sensors { CtBot::get_instance().get_sensors() };
        if (p_sensors) {
            if (data) {
                // std::cout << "CMD_SENS_RC5 received: data=0x" << std::hex << data << " addr=0x" << static_cast<uint16_t>(rc5_addr) << " cmd=0x"
                //           << static_cast<uint16_t>(rc5_cmd) << std::dec << " toggle=" << toggle << "\n";
                p_sensors->get_rc5().set_rc5(rc5_addr, rc5_cmd, toggle);
            }
        } else {
            // std::cout << "CMD_SENS_RC5 received: data=0x" << std::hex << data << std::dec << ", but no sensor found.\n";
        }
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

    receive_cmd_data();
}

void SimConnection::receive_cmd_data() {
    boost::asio::async_read_until(sock_, recv_bufer_, static_cast<char>(CommandCodes::CMD_STOPCODE),
        boost::bind(&SimConnection::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void SimConnection::handle_read(const boost::system::error_code& ec, std::size_t) {
    if (ec) {
        std::cerr << "SimConnection::handle_read(): connection to ct-Sim aborted: \"" << ec.message() << "\"\n";
        return;
    }

    evaluate_received_data();
}

void SimConnection::handle_write(const boost::system::error_code& ec) {
    if (ec) {
        std::cerr << "SimConnection::handle_write(): connection to ct-Sim aborted: \"" << ec.message() << "\"\n";
    }
}

void SimConnection::evaluate_received_data() {
    while (recv_bufer_.in_avail() > 0) {
        if (p_cmd_) {
            if (static_cast<std::streamsize>(p_cmd_->get_payload_size()) > recv_bufer_.in_avail()) {
                std::cerr << "SimConnection::evaluate_received_data(): not enough bytes for payload received: " << recv_bufer_.in_avail() << "\n";
                boost::asio::async_read(sock_, recv_bufer_, boost::asio::transfer_at_least(p_cmd_->get_payload_size() - recv_bufer_.in_avail()),
                    boost::bind(&SimConnection::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                return;
            }
            p_cmd_->append_payload(recv_bufer_, p_cmd_->get_payload_size());
        } else {
            if (recv_bufer_.in_avail() < static_cast<std::streamsize>(sizeof(CommandData))) {
                std::cerr << "SimConnection::evaluate_received_data(): not enough bytes received: " << recv_bufer_.in_avail() << "\n";
                boost::asio::async_read(sock_, recv_bufer_, boost::asio::transfer_at_least(sizeof(CommandData) - recv_bufer_.in_avail()),
                    boost::bind(&SimConnection::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                return;
            }

            try {
                p_cmd_ = std::make_unique<CommandNoCRC>(recv_bufer_);
            } catch (const std::exception& e) {
                std::cerr << "SimConnection::evaluate_received_data(): creating new command failed: \"" << e.what() << "\"\n";
                p_cmd_.reset();
                break;
            }

            assert(p_cmd_);
            if (p_cmd_->get_payload_size()) {
                if (recv_bufer_.in_avail() < static_cast<std::streamsize>(p_cmd_->get_payload_size())) {
                    std::cerr << "SimConnection::evaluate_received_data(): not enough bytes for payload received: " << recv_bufer_.in_avail() << "\n";
                    boost::asio::async_read(sock_, recv_bufer_, boost::asio::transfer_at_least(p_cmd_->get_payload_size() - recv_bufer_.in_avail()),
                        boost::bind(&SimConnection::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                    return;
                }
                p_cmd_->append_payload(recv_bufer_, p_cmd_->get_payload_size());
            }
        }

        if (!evaluate_cmd(*p_cmd_)) {
            std::cerr << "SimConnection::evaluate_received_data(): evaluate_cmd failed.\n";
        }

        p_cmd_.reset();
    }

    receive_cmd_data();
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
        boost::asio::async_write(
            sock_, boost::asio::buffer(&cmd.get_cmd(), sizeof(CommandData)), boost::bind(&SimConnection::handle_write, this, boost::asio::placeholders::error));
        if (cmd.get_payload_size()) {
            boost::asio::async_write(sock_, boost::asio::buffer(cmd.get_payload().data(), cmd.get_payload_size()),
                boost::bind(&SimConnection::handle_write, this, boost::asio::placeholders::error));
        }
        return true;
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
            boost::asio::async_write(sock_, boost::asio::buffer(&p_cmd->get_cmd(), sizeof(CommandData)),
                boost::bind(&SimConnection::handle_write, this, boost::asio::placeholders::error));
            if (p_cmd->get_payload_size()) {
                boost::asio::async_write(sock_, boost::asio::buffer(p_cmd->get_payload().data(), p_cmd->get_payload_size()),
                    boost::bind(&SimConnection::handle_write, this, boost::asio::placeholders::error));
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


uint8_t SimConnection::i2c_write_reg8(TwoWire* p_i2c, const uint8_t addr, const uint8_t reg, const uint8_t value) const {
    if (!p_i2c) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): p_i2c not initialized."));
        return 10;
    }

    p_i2c->beginTransmission(addr);
    if (p_i2c->write(reg) != 1) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): write() 1 failed."));
        return 20;
    }
    if (p_i2c->write(value) != 1) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): write() 2 failed."));
        return 30;
    }
    const uint8_t status { p_i2c->endTransmission(1U) };
    if (status) {
        arduino::Serial.print(PSTR("SimConnection::i2c_write_reg8(): endTransmission() failed: "));
        arduino::Serial.println(status, 10);
    }
    return status;
}

uint8_t SimConnection::i2c_write_reg8(TwoWire* p_i2c, const uint8_t addr, const uint16_t reg, const uint8_t value) const {
    if (!p_i2c) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): p_i2c_ not initialized."));
        return 10;
    }

    p_i2c->beginTransmission(addr);
    if (p_i2c->write(reg >> 8) != 1) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): write() 1 failed."));
        return 20;
    }
    if (p_i2c->write(reg & 0xff) != 1) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): write() 2 failed."));
        return 30;
    }
    if (p_i2c->write(value) != 1) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg8(): write() 3 failed."));
        return 40;
    }
    const uint8_t status { p_i2c->endTransmission(1U) };
    if (status) {
        arduino::Serial.print(PSTR("SimConnection::i2c_write_reg8(): endTransmission() failed: "));
        arduino::Serial.println(status, 10);
    }
    return status;
}

uint8_t SimConnection::i2c_write_reg16(TwoWire* p_i2c, const uint8_t addr, const uint8_t reg, const uint16_t value) const {
    if (!p_i2c) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg16(): p_i2c not initialized."));
        return 10;
    }

    p_i2c->beginTransmission(addr);
    if (p_i2c->write(reg) != 1) {
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg16(): write() 1 failed."));
        return 20;
    }
    if (p_i2c->write(value >> 8) != 1) { // high byte
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg16(): write() 2 failed."));
        return 4;
    }
    if (p_i2c->write(value & 0xff) != 1) { // low byte
        arduino::Serial.println(PSTR("SimConnection::i2c_write_reg16(): write() 3 failed."));
        return 4;
    }
    const uint8_t status { p_i2c->endTransmission(1U) };
    if (status) {
        arduino::Serial.print(PSTR("SimConnection::i2c_write_reg16(): endTransmission() failed: "));
        arduino::Serial.println(status, 10);
    }
    return status;
}

} /* namespace ctbot */

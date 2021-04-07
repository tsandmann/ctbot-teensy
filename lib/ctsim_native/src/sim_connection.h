
/**
 * @file    sim_connection.h
 * @brief   ct-Sim connection
 * @author  Timo Sandmann
 * @date    17.06.2018
 */

#pragma once

#include "command.h"
#include "../../../src/behavior/resource.h"

#include <cstdint>
#include <vector>
#include <list>
#include <functional>
#include <memory>
#include <map>
#include <thread>
#include <boost/asio.hpp>


class TwoWire;

namespace ctbot {

class SimConnection {
protected:
    boost::asio::streambuf recv_bufer_;
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket sock_;
    boost::asio::ip::tcp::resolver::iterator endpoint_it_;
    uint8_t bot_addr_;
    int16_t sim_time_ms_;
    TwoWire* p_i2c_range_;
    Resource<bool>* p_sim_res_;
    std::map<CommandCodes /*cmd*/, std::vector<std::function<bool(const CommandBase&)>> /*functions*/> commands_;
    std::unique_ptr<std::thread> p_io_thread_;
    std::unique_ptr<CommandBase> p_cmd_;

    void handle_connect(const boost::system::error_code& ec);
    void handle_read(const boost::system::error_code& ec, std::size_t size);
    void handle_write(const boost::system::error_code& ec);

    void receive_cmd_data();
    void evaluate_received_data();

    bool evaluate_cmd(const CommandBase& cmd);
    void register_cmd(const CommandCodes& cmd, decltype(commands_)::mapped_type::value_type func);


    uint8_t i2c_write_reg8(TwoWire* p_i2c, const uint8_t addr, const uint8_t reg, const uint8_t value) const;
    uint8_t i2c_write_reg8(TwoWire* p_i2c, const uint8_t addr, const uint16_t reg, const uint8_t value) const;
    uint8_t i2c_write_reg16(TwoWire* p_i2c, const uint8_t addr, const uint8_t reg, const uint16_t value) const;

public:
    SimConnection(const std::string& hostname, const std::string& port);
    ~SimConnection();

    bool send_cmd(CommandBase& cmd);
    bool send_cmd(std::vector<std::shared_ptr<CommandBase>>& cmds);
};

} /* namespace ctbot */

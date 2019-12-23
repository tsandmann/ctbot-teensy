
/**
 * @file    sim_connection.h
 * @brief   C't-Sim connection
 * @author  Timo Sandmann
 * @date    17.06.2018
 */

#pragma once

#include "command.h"

#include <cstdint>
#include <vector>
#include <list>
#include <functional>
#include <memory>
#include <map>
#include <thread>
#include <boost/asio.hpp>


namespace ctbot {

class I2C_Wrapper;

class SimConnection {
protected:
    boost::asio::streambuf recv_bufer_;
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket sock_;
    uint8_t bot_addr_;
    int16_t sim_time_ms_;
    uint64_t now_ms_;
    I2C_Wrapper* p_i2c_range_;
    std::map<CommandCodes /*cmd*/, std::vector<std::function<bool(const CommandBase&)>> /*functions*/> commands_;
    std::shared_ptr<std::thread> p_recv_thread_;

    bool evaluate_cmd(const CommandBase& cmd);
    void register_cmd(const CommandCodes& cmd, decltype(commands_)::mapped_type::value_type func);
    size_t receive_until(std::streambuf& buf, const char delim, const size_t maxsize);
    size_t send(const void* data, const size_t size);
    static void receiver_task(void* p_instance);

public:
    SimConnection(const std::string& hostname, const std::string& port);
    ~SimConnection();

    bool receive_sensor_data();
    bool receive_sensor_data(const CommandCodes cmd);
    bool send_cmd(CommandBase& cmd);
    bool send_cmd(std::vector<std::shared_ptr<CommandBase>>& cmds);
};

} /* namespace ctbot */

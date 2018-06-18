
/**
 * @file    command.cpp
 * @brief   Command management for c't-Sim connection
 * @author  Timo Sandmann
 * @date    17.06.2018
 */

#include "command.h"
#include <boost/asio/buffer.hpp>
#include <boost/asio/buffers_iterator.hpp>


namespace ctbot {

CommandBase::CommandBase(const CommandData& cmd_data) : data_ { cmd_data }, has_crc_ { false }, crc_ok_ { false } {}

CommandBase::CommandBase(const CommandCodes& cmd_code, const CommandCodes& subcmd_code, int16_t data_l, int16_t data_r, uint8_t from, uint8_t to) :
        data_ { cmd_code, subcmd_code, data_l, data_r, from }, has_crc_ { false }, crc_ok_ { false } {
    data_.to = to;
}

CommandBase::CommandBase(boost::asio::streambuf& buf) : has_crc_ { false }, crc_ok_ { false } {
    std::istream is { &buf };
    is.read(reinterpret_cast<char*>(&data_), sizeof(CommandData));
    uint8_t* ptr { reinterpret_cast<uint8_t*>(&data_) };
    while (data_.startCode != static_cast<uint8_t>(CommandCodes::CMD_STARTCODE) && is.good()) {
        char* ptr { reinterpret_cast<char*>(&data_) };
        std::memmove(ptr, &ptr[1], sizeof(CommandData) - 1);
        is.get(ptr[sizeof(CommandData) - 1]);
    }
    if (! valid()) {
        std::cerr << "CommandBase::CommandBase(): invalid command:\n" << *this << "\n";
        // FIXME: exception?
    }
}

void CommandBase::append_payload(boost::asio::streambuf& buf, const size_t len) {
    auto bufs { buf.data() };
    const auto n { len > MAX_PAYLOAD ? MAX_PAYLOAD : len };
    payload_.reserve(n);
    payload_.assign(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + static_cast<const ptrdiff_t>(n));
    buf.consume(n);
}

void CommandBase::append_crc(uint16_t crc) {
    has_crc_ = true;
    data_.to = static_cast<uint8_t>(crc & 0xff);
    data_.from = static_cast<uint8_t>((crc >> 8) & 0xff);
}

void CommandBase::add_payload(const void* payload, const size_t len) {
    data_.payload = len > MAX_PAYLOAD ? MAX_PAYLOAD : static_cast<uint8_t>(len);
    auto ptr { static_cast<const uint8_t*>(payload) };
    payload_.insert(payload_.begin(), ptr, ptr + data_.payload);
}

std::ostream& operator <<(std::ostream& os, const CommandBase& v) {
    os << "startCode='" << static_cast<char>(v.get_cmd().startCode) << "' (" << std::hex << std::showbase << static_cast<uint16_t>(v.get_cmd().startCode) << ")";
    if (v.get_cmd().startCode != static_cast<uint8_t>(CommandCodes::CMD_STARTCODE)) {
        os << " should be '" << static_cast<char>(CommandCodes::CMD_STARTCODE) << "' (" << std::hex << static_cast<uint16_t>(CommandCodes::CMD_STARTCODE) << ")";
    }
    os << "\n";
    os << "command='" << static_cast<char>(v.get_cmd().command) << "' (" << std::hex << static_cast<uint16_t>(v.get_cmd().command) << ")\n";
    os << "subcommand='" << static_cast<char>(v.get_cmd().subcommand) << "' (" << std::hex << static_cast<uint16_t>(v.get_cmd().subcommand) << ")\n";
    os << "direction=" << (v.get_cmd().direction == static_cast<uint8_t>(CommandDirection::CMD_REQUEST) ? "request" : "answer") << "\n";
    os << "seq=" << static_cast<uint16_t>(v.get_cmd().seq) << "\n";
    if (! v.has_crc_) {
        os << "from=" << static_cast<uint16_t>(v.get_cmd().from) << "\n";
        os << "to=" << static_cast<uint16_t>(v.get_cmd().to) << "\n";
    }
    os << "data_l=" << v.get_cmd().data_l << "\n";
    os << "data_r=" << v.get_cmd().data_r << "\n";
    os << "payload=" << static_cast<uint16_t>(v.get_cmd().payload) << "\n";
    os << "CRC='" << static_cast<char>(v.get_cmd().CRC) << "' (" << std::hex << static_cast<uint16_t>(v.get_cmd().CRC) << ")\n";
    if (v.get_cmd().CRC != static_cast<uint8_t>(CommandCodes::CMD_STOPCODE)) {
        os << " should be '" << static_cast<char>(CommandCodes::CMD_STOPCODE) << "' (" << std::hex << static_cast<uint16_t>(CommandCodes::CMD_STOPCODE) << ")\n";
    }
    if (v.has_crc_) {
        os << "crc16=" << std::hex << (static_cast<uint16_t>(v.get_cmd().to) | (static_cast<uint16_t>(v.get_cmd().from) << 8)) << "\n";
    }
    os << std::dec << "\n";
    return os;
}

} /* namespace ctbot */

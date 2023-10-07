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
 * @file    i2c_service.cpp
 * @brief   I2C FreeRTOS service
 * @author  Timo Sandmann
 * @date    25.03.2021
 */

#include "i2c_service.h"

#include "queue.h"

#include <atomic>


#undef printf_debug
#define printf_debug arduino::Serial.printf

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "driver/i2c_t4.h"
namespace I2C_NS = freertos;
#else
namespace I2C_NS = arduino;
#endif

decltype(I2C_Service::i2c_task_) I2C_Service::i2c_task_;
decltype(I2C_Service::i2c_queue_) I2C_Service::i2c_queue_;
decltype(I2C_Service::p_i2c_) I2C_Service::p_i2c_;
decltype(I2C_Service::init_) I2C_Service::init_;
decltype(I2C_Service::freq_) I2C_Service::freq_;

bool I2C_Service::init(uint8_t bus, uint32_t freq, uint8_t pin_sda, uint8_t pin_scl) {
    if (bus >= i2c_task_.size()) {
        if constexpr (DEBUG_) {
            printf_debug(PSTR("I2C_Service::init(%u, %u, %u, %u): invalid bus.\r\n"), bus, freq, pin_sda, pin_scl);
        }
        return false;
    }

    if (init_[bus] && freq_[bus] == freq) {
        return true;
    }

    if (init_[bus]) {
        p_i2c_[bus]->setClock(freq);
        return true;
    }

    if constexpr (DEBUG_) {
        printf_debug(PSTR("I2C_Service::init(%u, %u, %u, %u)...\r\n"), bus, freq, pin_sda, pin_scl);
    }

    switch (bus) {
        case 0: {
            p_i2c_[0] = I2C_NS::get_wire<0>();
            break;
        }

        case 1: {
            p_i2c_[1] = I2C_NS::get_wire<1>();
            break;
        }

        case 2: {
            p_i2c_[2] = I2C_NS::get_wire<2>();
            break;
        }

#ifdef WIRE_IMPLEMENT_WIRE3
        case 3: {
            p_i2c_[3] = I2C_NS::get_wire<3>();
            break;
        }
#endif // WIRE_IMPLEMENT_WIRE3

        default: {
            return false;
        }
    }

    if (pin_sda != 255) {
        p_i2c_[bus]->setSDA(pin_sda);
    }
    if (pin_scl != 255) {
        p_i2c_[bus]->setSCL(pin_scl);
    }

    freq_[bus] = freq;
    reset(bus);

    if (!i2c_task_[bus]) {
        i2c_queue_[bus] = ::xQueueCreate(TRANSFER_QUEUE_SIZE_, sizeof(I2C_Transfer*));
        if (!i2c_queue_[bus]) {
            init_[bus] = false;
            return false;
        }

        uintptr_t x { bus };
        void* param { reinterpret_cast<void*>(x) };
        char tmp[10] { "I2C Svc" };
        tmp[7] = ' ';
        tmp[8] = bus + 0x31;
        tmp[9] = 0;
        if (::xTaskCreate(run, tmp, 512, param, 8, &i2c_task_[bus]) != pdTRUE) {
            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::init(): xTaskCreate() failed\r\n"));
            }
            ::vQueueDelete(i2c_queue_[bus]);
            init_[bus] = false;
            return false;
        }
    }

    init_[bus] = true;

    if constexpr (DEBUG_) {
        printf_debug(PSTR("I2C_Service::init(%u, %u, %u, %u) done\r\n"), bus, freq, pin_sda, pin_scl);
    }

    return true;
}

void I2C_Service::reset(uint8_t bus) {
    p_i2c_[bus]->begin();
    p_i2c_[bus]->setClock(freq_[bus]);
}

void I2C_Service::finish_transfer(bool success, I2C_Transfer* transfer) {
    auto p_data { transfer };
    if (p_data->callback) {
        p_data->callback(success, &p_data);
    }
    delete p_data;
}

void I2C_Service::finish_transfer(bool success, I2C_Transfer* transfer, [[maybe_unused]] const uint32_t id) {
    if (DEBUG_ && transfer->callback) {
        printf_debug(PSTR("I2C_Service::finish_transfer(): callback for transfer %u addr=0x%x, err=%u\r\n"), id, transfer->addr, transfer->error);
    }
    finish_transfer(success, transfer);
}

void I2C_Service::run(void* param) {
    const uintptr_t bus { reinterpret_cast<uintptr_t>(param) };
    if constexpr (DEBUG_) {
        printf_debug(PSTR("I2C_Service::run(): bus=%u\r\n"), bus);
    }

    I2C_Transfer* transfer {};
    uint8_t buffer[4];
    uint32_t id {};

    while (true) {
        if (::xQueueReceive(i2c_queue_[bus], &transfer, portMAX_DELAY) == pdPASS) {
            if (!transfer) {
                continue;
            }
            if constexpr (DEBUG_) {
                printf_debug(PSTR("\r\nI2C_Service::run(): transfer %u for 0x%x received:\r\n"), ++id, transfer->addr);
                if (transfer->size1) {
                    printf_debug(PSTR("\ttype1=%c\tsize1=%u\tdata1=0x%x\r\n"), transfer->type1 ? 'r' : 'w', transfer->size1,
                        transfer->type1 == 0 ? transfer->data1.data : 0);
                }
                if (transfer->size2) {
                    printf_debug(PSTR("\ttype2=%c\tsize2=%u\tdata2=0x%x\r\n"), transfer->type2 ? 'r' : 'w', transfer->size2,
                        transfer->type2 == 0 ? transfer->data2.data : 0);
                }
            }

            p_i2c_[bus]->beginTransmission(transfer->addr);
            if (transfer->type1 == 0) {
                /* 1st transfer write */
                const auto to_send { transfer->size1 };
                if (to_send <= sizeof(I2C_Transfer::data1.data)) {
                    uint8_t* ptr { buffer };
                    for (int i { to_send - 1 }; i >= 0; --i) {
                        *ptr = static_cast<uint8_t>(transfer->data1.data >> (i << 3));
                        if constexpr (DEBUG_) {
                            // printf_debug(PSTR("I2C_Service::run(): write buffer[%u]=0x%x\r\n"), ptr - buffer, *ptr);
                        }
                        ++ptr;
                    }
                    transfer->data1.ptr = buffer;
                }

                if (p_i2c_[bus]->write(transfer->data1.ptr, to_send) != to_send) {
                    if constexpr (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): write() 1 FAILED\r\n"));
                    }
                    transfer->error = I2C_Error::WRITE_1_FAILURE;
                    finish_transfer(false, transfer, id);
                    continue;
                }

                if (!transfer->size2 || transfer->type2) {
                    const auto status { p_i2c_[bus]->endTransmission(transfer->size2 && transfer->type2 ? 0 : 1) };
                    if (status) {
                        if constexpr (DEBUG_) {
                            printf_debug(PSTR("I2C_Service::run(): endTransmission() FAILED: %u\r\n"), status);
                        }
                        transfer->error = I2C_Error::END_TRANSMISSION_1_FAILURE;
                        finish_transfer(false, transfer, id);
                        continue;
                    }
                }
            } else if (!transfer->size1) {
                /* just scan for address */
                if (p_i2c_[bus]->endTransmission(1)) {
                    transfer->error = I2C_Error::END_TRANSMISSION_1_FAILURE;
                    finish_transfer(false, transfer, id);
                    continue;
                }
            } else {
                /* 1st transfer read with size > 0 */
                transfer->error = I2C_Error::NOT_IMPLEMENTED;
                configASSERT(false); // TODO: not implemented
            }

            if (transfer->size2 && transfer->type2 == 0) {
                /* 2nd transfer write */
                const auto to_send { transfer->size2 };
                if (to_send <= sizeof(I2C_Transfer::data2.data)) {
                    uint8_t* ptr { buffer };
                    for (auto i { to_send - 1 }; i >= 0; --i) {
                        *ptr = static_cast<uint8_t>(transfer->data2.data >> (i << 3));
                        if constexpr (DEBUG_) {
                            // printf_debug(PSTR("I2C_Service::run(): write buffer[%u]=0x%x\r\n"), ptr - buffer, *ptr);
                        }
                        ++ptr;
                    }
                    transfer->data2.ptr = buffer;
                }

                if (p_i2c_[bus]->write(transfer->data2.ptr, to_send) != to_send) {
                    if constexpr (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): write() 2 FAILED\r\n"));
                    }
                    transfer->error = I2C_Error::WRITE_2_FAILURE;
                    finish_transfer(false, transfer, id);
                    continue;
                }

                const auto status { p_i2c_[bus]->endTransmission(1) };
                if (status) {
                    if constexpr (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): endTransmission() FAILED: %u\r\n"), status);
                    }
                    transfer->error = I2C_Error::END_TRANSMISSION_2_FAILURE;
                    finish_transfer(false, transfer, id);
                    continue;
                }
            } else if (transfer->size2) {
                /* 2nd transfer read */
                const auto to_read { transfer->size2 };
                if (p_i2c_[bus]->requestFrom(static_cast<uint8_t>(transfer->addr), to_read, static_cast<uint8_t>(1)) != to_read) {
                    if constexpr (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): requestFrom() FAILED\r\n"));
                    }
                    transfer->error = I2C_Error::REQUEST_FROM_FAILURE;
                    finish_transfer(false, transfer, id);
                    continue;
                }

                if (to_read <= sizeof(I2C_Transfer::data2.data)) {
                    if (p_i2c_[bus]->readBytes(buffer, to_read) != to_read) {
                        transfer->error = I2C_Error::READ_FAILURE;
                        finish_transfer(false, transfer, id);
                        continue;
                    }

                    transfer->data2.data = 0;
                    uint8_t* ptr { buffer };
                    for (auto i { to_read - 1 }; i >= 0; --i) {
                        transfer->data2.data |= static_cast<uint32_t>(*ptr) << (i << 3);
                        if constexpr (DEBUG_) {
                            // printf_debug(PSTR("I2C_Service::run(): read buffer[%u]=0x%x\r\n"), ptr - buffer, *ptr);
                        }
                        ++ptr;
                    }
                    if constexpr (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): rx_data.data=0x%x\r\n"), transfer->data2.data);
                    }
                } else {
                    /* to_read > sizeof(I2C_Transfer::data2.data) */
                    if (p_i2c_[bus]->readBytes(transfer->data2.ptr, to_read) != to_read) {
                        if constexpr (DEBUG_) {
                            printf_debug(PSTR("I2C_Service::run(): readBytes() FAILED\r\n"));
                        }
                        transfer->error = I2C_Error::READ_FAILURE;
                        finish_transfer(false, transfer, id);
                        continue;
                    }
                }
            }

            finish_transfer(true, transfer, id);
            if constexpr (DEBUG_) {
                printf_debug(PSTR("\r\nI2C_Service::run(): transfer %u completed\r\n\n"), id);
            }
        }
    }
}

I2C_Service::I2C_Service(uint8_t bus, uint32_t freq, uint8_t pin_sda, uint8_t pin_scl) : bus_ { bus } {
    init(bus, freq, pin_sda, pin_scl);
}

I2C_Service::I2C_Error I2C_Service::read_reg(
    uint16_t addr, std::unsigned_integral auto reg, std::integral auto& data, std::function<void(bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::read_reg<>(): invalid reg size");
    static_assert(sizeof(data) <= sizeof(I2C_Transfer::data2.data), "I2C_Service::read_reg<>(): invalid data size");

    std::atomic<bool> transfer_done;
    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg), false, sizeof(data), true } };
    transfer->data1.data = reg;

    I2C_Error ret;
    if (callback) {
        transfer->callback = callback;
        ret = I2C_Error::SUCCESS;
    } else {
        transfer_done = false;
        transfer->callback = [&data, &transfer_done, &ret](bool done, I2C_Transfer** p_transfer) {
            if (done) {
                data = static_cast<std::remove_reference<decltype(data)>::type>((*p_transfer)->data2.data);
            }
            ret = (*p_transfer)->error;
            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::read_reg<%u, %u>() callback, data=0x%x, err=%u\r\n"), sizeof(reg), sizeof(data), data, ret);
            }
            transfer_done = true;
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, pdMS_TO_TICKS(TRANSFER_QUEUE_TIMEOUT_MS_)) != pdTRUE) {
        return I2C_Error::QUEUE_FULL;
    }

    if (!callback) {
        const auto start { ::xTaskGetTickCount() };
        while (!transfer_done) {
            if (::xTaskGetTickCount() - start > pdMS_TO_TICKS(DEFAULT_CALLBACK_TIMEOUT_MS_)) {
                return I2C_Error::TIMEOUT;
            }
            ::vTaskDelay(1);
        }
        if constexpr (DEBUG_) {
            // printf_debug(PSTR("I2C_Service::read_reg<%u, %u>(): done, ret=%u, data=%u\r\n"), sizeof(REG), sizeof(DATA), ret, data);
        }
    }

    return ret;
}

template I2C_Service::I2C_Error I2C_Service::read_reg<uint8_t, uint8_t>(uint16_t, uint8_t, uint8_t&, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::read_reg<uint16_t, uint8_t>(uint16_t, uint16_t, uint8_t&, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::read_reg<uint8_t, uint16_t>(uint16_t, uint8_t, uint16_t&, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::read_reg<uint16_t, uint16_t>(uint16_t, uint16_t, uint16_t&, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::read_reg<uint8_t, uint32_t>(uint16_t, uint8_t, uint32_t&, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::read_reg<uint16_t, uint32_t>(uint16_t, uint16_t, uint32_t&, std::function<void(bool, I2C_Transfer**)>) const;

I2C_Service::I2C_Error I2C_Service::read_bytes(
    uint16_t addr, std::unsigned_integral auto reg_addr, uint8_t* p_data, uint8_t length, std::function<void(bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg_addr) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::read_bytes<>(): invalid reg_addr size");

    if (length > 32) {
        return I2C_Error::INVALID_PARAMETERS;
    }

    switch (length) {
        case 0: {
            return I2C_Error::SUCCESS;
        }
        case 1: {
            uint8_t tmp;
            const auto ret { read_reg(addr, reg_addr, tmp) };
            p_data[0] = tmp;
            return ret;
        }
        case 2: {
            uint16_t tmp;
            const auto ret { read_reg(addr, reg_addr, tmp) };
            p_data[0] = static_cast<uint8_t>(tmp >> 8);
            p_data[1] = static_cast<uint8_t>(tmp);
            return ret;
        }
        case 3: {
            return I2C_Error::NOT_IMPLEMENTED; // TODO: not implemented
        }
        case 4: {
            uint32_t tmp;
            const auto ret { read_reg(addr, reg_addr, tmp) };
            p_data[0] = static_cast<uint8_t>(tmp >> 24);
            p_data[1] = static_cast<uint8_t>(tmp >> 16);
            p_data[2] = static_cast<uint8_t>(tmp >> 8);
            p_data[3] = static_cast<uint8_t>(tmp);
            return ret;
        }
    }

    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg_addr), false, length, true } };
    transfer->data1.data = reg_addr;
    transfer->data2.ptr = p_data;

    I2C_Error ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer->caller = ::xTaskGetCurrentTaskHandle();
        transfer->callback = [&ret]([[maybe_unused]] bool done, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;
            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::read_bytes(): callback, done=%u, err=%u\r\n"), done, static_cast<uint8_t>(ret));
            }
            ::xTaskNotifyGive((*p_transfer)->caller);
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, pdMS_TO_TICKS(TRANSFER_QUEUE_TIMEOUT_MS_)) != pdTRUE) {
        return I2C_Error::QUEUE_FULL;
    }

    if (callback) {
        return I2C_Error::SUCCESS;
    }

    ::ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(READ_BYTES_TIMEOUT_MS_PER_BYTE_));

    if constexpr (DEBUG_) {
        // printf_debug(PSTR("I2C_Service::read_bytes(): done, err=%u\r\n"), static_cast<uint8_t>(ret));
    }

    return ret;
}

template I2C_Service::I2C_Error I2C_Service::read_bytes<uint8_t>(uint16_t, uint8_t, uint8_t*, uint8_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::read_bytes<uint16_t>(uint16_t, uint16_t, uint8_t*, uint8_t, std::function<void(bool, I2C_Transfer**)>) const;

I2C_Service::I2C_Error I2C_Service::write_reg(
    uint16_t addr, std::unsigned_integral auto reg, std::integral auto data, std::function<void(bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::write_reg<>(): invalid reg size");
    static_assert(sizeof(data) <= sizeof(I2C_Transfer::data2.data), "I2C_Service::write_reg<>(): invalid data size");

    std::atomic<bool> transfer_done;
    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg), false, sizeof(data), false } };
    transfer->data1.data = reg;
    transfer->data2.data = data;

    I2C_Error ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer_done = false;
        transfer->callback = [&transfer_done, &ret](bool, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;
            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::write_reg<%u, %u>() callback done, err=%u\r\n"), sizeof(reg), sizeof(data), ret);
            }
            transfer_done = true;
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, pdMS_TO_TICKS(TRANSFER_QUEUE_TIMEOUT_MS_)) != pdTRUE) {
        if constexpr (DEBUG_) {
            printf_debug(PSTR("I2C_Service::write_reg<%u, %u>() transfer queueing FAILED\r\n"), sizeof(reg), sizeof(data));
        }
        return I2C_Error::QUEUE_FULL;
    }

    if (!callback) {
        const auto start { ::xTaskGetTickCount() };
        while (!transfer_done) {
            if (::xTaskGetTickCount() - start > pdMS_TO_TICKS(DEFAULT_CALLBACK_TIMEOUT_MS_)) {
                return I2C_Error::TIMEOUT;
            }
            ::vTaskDelay(1);
        }
        if constexpr (DEBUG_) {
            // printf_debug(PSTR("I2C_Service::write_reg<%u, %u>(): done, ret=%u\r\n"), sizeof(reg), sizeof(data), ret);
        }
    }

    return ret;
}

template I2C_Service::I2C_Error I2C_Service::write_reg<uint8_t, uint8_t>(uint16_t, uint8_t, uint8_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::write_reg<uint16_t, uint8_t>(uint16_t, uint16_t, uint8_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::write_reg<uint8_t, uint16_t>(uint16_t, uint8_t, uint16_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::write_reg<uint16_t, uint16_t>(uint16_t, uint16_t, uint16_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::write_reg<uint8_t, uint32_t>(uint16_t, uint8_t, uint32_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::write_reg<uint16_t, uint32_t>(uint16_t, uint16_t, uint32_t, std::function<void(bool, I2C_Transfer**)>) const;

I2C_Service::I2C_Error I2C_Service::write_bytes(
    uint16_t addr, std::unsigned_integral auto reg_addr, const uint8_t* p_data, uint8_t length, std::function<void(bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg_addr) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::write_bytes<>(): invalid reg_addr size");

    if (length > 32) {
        return I2C_Error::INVALID_PARAMETERS;
    }

    switch (length) {
        case 0: {
            return I2C_Error::SUCCESS;
        }
        case 1: {
            uint8_t tmp { *p_data };
            return write_reg(addr, reg_addr, tmp);
        }
        case 2: {
            uint16_t tmp { p_data[1] };
            tmp |= p_data[0] << 8;
            return write_reg(addr, reg_addr, tmp);
        }
        case 3: {
            return I2C_Error::NOT_IMPLEMENTED; // TODO: not implemented
        }
        case 4: {
            uint32_t tmp { p_data[3] };
            tmp |= p_data[2] << 8;
            tmp |= p_data[1] << 16;
            tmp |= p_data[0] << 24;
            return write_reg(addr, reg_addr, tmp);
        }
    }

    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg_addr), false, length, false } };
    transfer->data1.data = reg_addr;
    transfer->data2.ptr = const_cast<uint8_t*>(p_data);

    I2C_Error ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer->caller = ::xTaskGetCurrentTaskHandle();
        transfer->callback = [&ret]([[maybe_unused]] bool done, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;
            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::write_bytes<%u>(): callback, done=%u, err=%u\r\n"), sizeof(reg_addr), done, ret);
            }
            ::xTaskNotifyGive((*p_transfer)->caller);
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, pdMS_TO_TICKS(TRANSFER_QUEUE_TIMEOUT_MS_)) != pdTRUE) {
        return I2C_Error::QUEUE_FULL;
    }

    if (callback) {
        return I2C_Error::SUCCESS;
    }

    ::ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WRITE_BYTES_TIMEOUT_MS_PER_BYTE_ * length));

    if constexpr (DEBUG_) {
        // printf_debug(PSTR("I2C_Service::write_bytes<%u>(): done, ret=%u\r\n"), static_cast<uint8_t>(ret), sizeof(reg));
    }

    return ret;
}

template I2C_Service::I2C_Error I2C_Service::write_bytes<uint8_t>(uint16_t, uint8_t, const uint8_t*, uint8_t, std::function<void(bool, I2C_Transfer**)>) const;
template I2C_Service::I2C_Error I2C_Service::write_bytes<uint16_t>(
    uint16_t, uint16_t, const uint8_t*, uint8_t, std::function<void(bool, I2C_Transfer**)>) const;


template <typename DATA>
I2C_Service::I2C_Error I2C_Service::set_bits_internal(uint16_t addr, std::unsigned_integral auto reg, uint32_t bit_mask, uint32_t values) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::set_bits_internal<>(): invalid reg size");
    static_assert(sizeof(DATA) <= sizeof(I2C_Transfer::data2.data), "I2C_Service::set_bits_internal<>(): invalid DATA size");

    if constexpr (DEBUG_) {
        printf_debug(PSTR("I2C_Service::set_bits_internal<%u, %u>(0x%x, 0x%x, 0x%x, 0x%x)\r\n"), sizeof(reg), sizeof(DATA), addr, reg, bit_mask, values);
    }

    DATA data;
    I2C_Error ret;
    std::atomic<bool> transfer_done {};
    auto read_callback { [&data, &ret, &transfer_done, this, addr, reg, bit_mask, values](bool done, I2C_Transfer** p_transfer) {
        if (done) {
            data = static_cast<DATA>((*p_transfer)->data2.data);

            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::set_bits_internal<%u, %u>() read_callback: data=0x%x\r\n"), sizeof(reg), sizeof(DATA), data);
            }

            data = (data & (~bit_mask)) | (values & bit_mask);

            ret = write_reg(addr, reg, data, [&transfer_done, data](bool, [[maybe_unused]] I2C_Transfer** p_transfer) {
                if constexpr (DEBUG_) {
                    printf_debug(PSTR("I2C_Service::set_bits_internal<%u, %u>() write_callback done, data=0x%x, err=%u\r\n"), sizeof(reg), sizeof(DATA), data,
                        (*p_transfer)->error);
                }
                transfer_done = true;
            });
        } else {
            ret = (*p_transfer)->error;
            if constexpr (DEBUG_) {
                printf_debug(
                    PSTR("I2C_Service::set_bits_internal<%u, %u>() read_reg<>() FAILED, err=%u\r\n"), sizeof(reg), sizeof(DATA), static_cast<uint8_t>(ret));
            }
        }
    } };

    if (read_reg(addr, reg, data, read_callback) != I2C_Error::SUCCESS) {
        if constexpr (DEBUG_) {
            printf_debug(
                PSTR("I2C_Service::set_bits_internal<%u, %u>(): read_reg<>() FAILED, ret=%u\r\n"), sizeof(reg), sizeof(DATA), static_cast<uint8_t>(ret));
        }
        return I2C_Error::READ_FAILURE;
    }

    const auto start { ::xTaskGetTickCount() };
    while (!transfer_done) {
        if (::xTaskGetTickCount() - start > pdMS_TO_TICKS(DEFAULT_CALLBACK_TIMEOUT_MS_)) {
            return I2C_Error::TIMEOUT;
        }
        ::vTaskDelay(1);
    }
    if constexpr (DEBUG_) {
        if (ret != I2C_Error::SUCCESS) {
            // printf_debug(PSTR("I2C_Service::set_bits_internal<%u, %u>(): ret=%u\r\n"), sizeof(REG), sizeof(DATA), static_cast<uint8_t>(ret));
        }
        printf_debug(PSTR("I2C_Service::set_bits_internal<%u, %u>(0x%x, 0x%x, %u, %u) done\r\n"), sizeof(reg), sizeof(DATA), addr, reg, bit_mask, values);
    }

    return ret;
}

I2C_Service::I2C_Error I2C_Service::set_bit(uint16_t addr, std::unsigned_integral auto reg, uint8_t bit, bool value) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::set_bit<>(): invalid reg size");

    if (bit < 8) {
        return set_bits_internal<uint8_t>(addr, reg, 1 << bit, static_cast<uint8_t>(value) << bit);
    } else if (bit < 16) {
        return set_bits_internal<uint16_t>(addr, reg, 1 << bit, static_cast<uint16_t>(value) << bit);
    } else if (bit < 32) {
        return set_bits_internal<uint32_t>(addr, reg, 1 << bit, static_cast<uint32_t>(value) << bit);
    }

    return I2C_Error::INVALID_PARAMETERS;
}

template I2C_Service::I2C_Error I2C_Service::set_bit<uint8_t>(uint16_t, uint8_t, uint8_t, bool) const;
template I2C_Service::I2C_Error I2C_Service::set_bit<uint16_t>(uint16_t, uint16_t, uint8_t, bool) const;


I2C_Service::I2C_Error I2C_Service::set_bits(uint16_t addr, std::unsigned_integral auto reg, uint32_t bit_mask, uint32_t values) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::set_bits<>(): invalid reg size");

    if (bit_mask < (1 << 8)) {
        return set_bits_internal<uint8_t>(addr, reg, bit_mask, values);
    } else if (bit_mask < (1 << 16)) {
        return set_bits_internal<uint16_t>(addr, reg, bit_mask, values);
    } else {
        return set_bits_internal<uint32_t>(addr, reg, bit_mask, values);
    }

    return I2C_Error::INVALID_PARAMETERS;
}

template I2C_Service::I2C_Error I2C_Service::set_bits<uint8_t>(uint16_t, uint8_t, uint32_t, uint32_t) const;
template I2C_Service::I2C_Error I2C_Service::set_bits<uint16_t>(uint16_t, uint16_t, uint32_t, uint32_t) const;


I2C_Service::I2C_Error I2C_Service::test(uint16_t addr, std::function<void(bool, I2C_Transfer**)> callback) const {
    I2C_Transfer* transfer { new I2C_Transfer { addr } };

    I2C_Error ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer->caller = ::xTaskGetCurrentTaskHandle();
        transfer->callback = [&ret]([[maybe_unused]] bool done, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;

            if constexpr (DEBUG_) {
                printf_debug(PSTR("I2C_Service::test(): callback, done=%u\r\n"), done);
            }

            ::xTaskNotifyGive((*p_transfer)->caller);
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, pdMS_TO_TICKS(TRANSFER_QUEUE_TIMEOUT_MS_)) != pdTRUE) {
        return I2C_Error::QUEUE_FULL;
    }

    if (callback) {
        return I2C_Error::SUCCESS;
    }

    ::ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TEST_TIMEOUT_MS_));

    return ret;
}

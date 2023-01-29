/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2022 Timo Sandmann
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
 * @file    fs_service.cpp
 * @brief   FreeRTOS service for filesystem on SD card
 * @author  Timo Sandmann
 * @date    19.11.2022
 */

#include "fs_service.h"

#include "task.h"
#include "queue.h"

#include "SD.h"


FS_Service::FS_Service(SDClass& fs) : fs_ { fs } {}

FLASHMEM bool FS_Service::init() {
    if (!job_queue_) {
        job_queue_ = ::xQueueCreate(WORKER_QUEUE_SIZE_, sizeof(queue_t*));
    }

    if (!worker_) {
        if (::xTaskCreate(run, PSTR("FS Svc"), WORKER_TASK_STACK_SIZE_ / sizeof(StackType_t), nullptr, WORKER_TASK_PRIORITY_, &worker_) != pdTRUE) {
            if constexpr (DEBUG_) {
                Serial.print(PSTR("FS_Service::init(): xTaskCreate() failed\r\n"));
            }
            ::vQueueDelete(job_queue_);
            job_queue_ = nullptr;
        }
    }
    configASSERT(worker_);
    configASSERT(job_queue_);

    if constexpr (DEBUG_) {
        Serial.print(PSTR("FS_Service::init() done\r\n"));
    }

    return true;
}

FLASHMEM void FS_Service::run(void*) {
    queue_t* p_operation;

    while (true) {
        if (::xQueueReceive(job_queue_, &p_operation, portMAX_DELAY) == pdPASS) {
            if (!p_operation) {
                continue;
            }

            if (auto p_file_op = std::get_if<FileOperation*>(p_operation)) {
                auto p_file_operation { *p_file_op };
                if constexpr (DEBUG_) {
                    Serial.printf(PSTR("FS_Service::run(): FILE operation received, p_file=0x%x\r\n"), p_file_operation->p_file_);
                }

                p_file_operation->result_ = p_file_operation->operation_(p_file_operation->p_file_);

                if constexpr (DEBUG_) {
                    Serial.print(PSTR("FS_Service::run(): FILE operation done.\r\n"));
                }

                if (p_file_operation->callback_ && !p_file_operation->canceled_) {
                    p_file_operation->callback_(p_file_operation);
                }

                delete p_file_operation;
            } else if (auto p_fs_op = std::get_if<FSOperation*>(p_operation)) {
                auto p_fs_operation { *p_fs_op };
                if constexpr (DEBUG_) {
                    Serial.printf(PSTR("FS_Service::run(): FS operation received, p_fs=0x%x\r\n"), p_fs_operation->p_fs_);
                }

                p_fs_operation->result_ = p_fs_operation->operation_(p_fs_operation->p_fs_);

                if constexpr (DEBUG_) {
                    Serial.print(PSTR("FS_Service::run(): FS operation done.\r\n"));
                }

                if (p_fs_operation->callback_ && !p_fs_operation->canceled_) {
                    p_fs_operation->callback_(p_fs_operation);
                }

                delete p_fs_operation;
            }

            delete p_operation;
        }
    }
}

bool FS_Service::begin(SdioConfig&& sdio_config) const {
    if constexpr (DEBUG_) {
        Serial.print(PSTR("FS_Service::begin()\r\n"));
    }
    return fs_.sdfs.begin(sdio_config) && init();
}

void FS_Service::set_priority(uint8_t new_priority) const {
    if (new_priority != WORKER_TASK_PRIORITY_) {
        ::vTaskPrioritySet(worker_, new_priority);
    }
}

void FS_Service::reset_priority() const {
    ::vTaskPrioritySet(worker_, WORKER_TASK_PRIORITY_);
}

int FS_Service::read(FileWrapper& file, void* buf, size_t size, file_callback_t callback) const {
    int result { -1 };

    auto p_operation { new FileOperation { [buf, size](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("READ operation: p_file=0x%x, buf=0x%x, size=%u\r\n"), p_file, buf, size);
                                              }
                                              const auto tmp { p_file->fs_file_.read(buf, size) };
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("READ operation: tmp=%d\r\n"), tmp);
                                              }

                                              file_result_t res { tmp };
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("READ operation: res=%d\r\n"), std::get<int>(res));
                                              }

                                              return res;
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<int>(p_operation->result_);
                       if constexpr (DEBUG_) {
                           Serial.printf(PSTR("READ operation: callback for 0x%x called, result=%d\r\n"), p_operation->caller_, result);
                       }
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_);

    if (DEBUG_ && result < 0) {
        Serial.printf(PSTR("READ operation for 0x%x failed, &file=0x%x, size=%u, result=%d\r\n"), p_operation->caller_, &file, size, result);
    }

    return result;
}

int FS_Service::peek(FileWrapper& file, file_callback_t callback) const {
    int result {};

    auto p_operation { new FileOperation { [](FileWrapper* p_file) { return p_file->fs_file_.peek(); },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<int>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = -1;
    }

    return result;
}

int FS_Service::available(FileWrapper& file, file_callback_t callback) const {
    int result {};

    auto p_operation { new FileOperation { [](FileWrapper* p_file) { return p_file->fs_file_.available(); },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<int>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = -1;
    }

    return result;
}

size_t FS_Service::write(FileWrapper& file, const void* buf, size_t size, file_callback_t callback) const {
    size_t result {};

    auto p_operation { new FileOperation { [buf, size](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("WRITE operation: p_file=0x%x, buf=0x%x, size=%u\r\n"), p_file, buf, size);
                                              }
                                              return p_file->fs_file_.write(buf, size);
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<size_t>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_);

    return result;
}

void FS_Service::flush(FileWrapper& file, file_callback_t callback) const {
    auto p_operation { new FileOperation { [](FileWrapper* p_file) {
                                              p_file->fs_file_.flush();
                                              return true;
                                          },
        callback ? callback : [](FileOperation* p_operation) { ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_); },
        ::xTaskGetCurrentTaskHandle(), file } };

    schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_);
}

bool FS_Service::seek(FileWrapper& file, int64_t pos, int mode, file_callback_t callback) const {
    bool result { true };

    auto p_operation { new FileOperation { [pos, mode](FileWrapper* p_file) {
                                              switch (mode) {
                                                  case SeekSet: return p_file->fs_file_.seekSet(pos);
                                                  case SeekCur: return p_file->fs_file_.seekCur(pos);
                                                  case SeekEnd: return p_file->fs_file_.seekEnd(pos);
                                                  default: return false;
                                              }
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = false;
    }

    return result;
}

bool FS_Service::truncate(FileWrapper& file, uint64_t size, file_callback_t callback) const {
    bool result { true };

    auto p_operation { new FileOperation { [size](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("TRUNCATE operation: p_file=0x%x, size=%u\r\n"), p_file, size);
                                              }
                                              return p_file->fs_file_.truncate(size);
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = false;
    }

    return result;
}

void FS_Service::rewind_directory(FileWrapper& file, file_callback_t callback) const {
    auto p_operation { new FileOperation { [](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("REWIND_DIR operation: p_file=0x%x\r\n"), p_file);
                                              }
                                              p_file->fs_file_.rewindDirectory();
                                              return true;
                                          },
        callback ? callback : [](FileOperation* p_operation) { ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_); },
        ::xTaskGetCurrentTaskHandle(), file } };

    schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_);
}

void FS_Service::close(FileWrapper& file, file_callback_t callback) const {
    if (!file.isOpen()) {
        return;
    }
    auto p_operation { new FileOperation { [](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("CLOSE operation: p_file=0x%x\r\n"), p_file);
                                              }
                                              return p_file->fs_file_.close();
                                          },
        callback ? callback : [](FileOperation* p_operation) { ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_); },
        ::xTaskGetCurrentTaskHandle(), file } };

    schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_);
}

File FS_Service::open_next_file(FileWrapper& file, uint8_t mode, FileWrapper** p_file_wrapper, file_callback_t) const {
    File f {};

    auto p_operation { new FSOperation { [&file, mode, p_file_wrapper, this](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("open_next_file operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return open_next(file.fs_file_, mode, pdTICKS_TO_US(file.timeout_ticks_), p_file_wrapper);
                                        },
        [&f](FSOperation* p_operation) {
            f = std::get<File>(p_operation->result_);
            ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
        },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    schedule_operation(p_operation, false, file.timeout_ticks_);

    return f;
}

bool FS_Service::get_create_time(FileWrapper& file, DateTimeFields& tm, file_callback_t callback) const {
    bool result { true };

    auto p_operation { new FileOperation { [&tm](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("GET_CREATE_TIME operation: p_file=0x%x\r\n"), p_file);
                                              }

                                              uint16_t fat_date, fat_time;
                                              if (!p_file->fs_file_.getCreateDateTime(&fat_date, &fat_time)) {
                                                  return false;
                                              }
                                              if (fat_date == 0 && fat_time == 0) {
                                                  return false;
                                              }

                                              tm.sec = FS_SECOND(fat_time);
                                              tm.min = FS_MINUTE(fat_time);
                                              tm.hour = FS_HOUR(fat_time);
                                              tm.mday = FS_DAY(fat_date);
                                              tm.mon = FS_MONTH(fat_date) - 1;
                                              tm.year = FS_YEAR(fat_date) - 1'900;

                                              return true;
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = false;
    }

    return result;
}

bool FS_Service::get_modify_time(FileWrapper& file, DateTimeFields& tm, file_callback_t callback) const {
    bool result { true };

    auto p_operation { new FileOperation { [&tm](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("GET_MODIFY_TIME operation: p_file=0x%x\r\n"), p_file);
                                              }

                                              uint16_t fat_date, fat_time;
                                              if (!p_file->fs_file_.getModifyDateTime(&fat_date, &fat_time)) {
                                                  return false;
                                              }
                                              if (fat_date == 0 && fat_time == 0) {
                                                  return false;
                                              }

                                              tm.sec = FS_SECOND(fat_time);
                                              tm.min = FS_MINUTE(fat_time);
                                              tm.hour = FS_HOUR(fat_time);
                                              tm.mday = FS_DAY(fat_date);
                                              tm.mon = FS_MONTH(fat_date) - 1;
                                              tm.year = FS_YEAR(fat_date) - 1'900;

                                              return true;
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = false;
    }

    return result;
}

bool FS_Service::set_create_time(FileWrapper& file, const DateTimeFields& tm, file_callback_t callback) const {
    bool result { true };

    auto p_operation { new FileOperation { [&tm](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("SET_CREATE_TIME operation: p_file=0x%x\r\n"), p_file);
                                              }

                                              if (tm.year < 80 || tm.year > 207) {
                                                  return false;
                                              }

                                              return p_file->fs_file_.timestamp(T_CREATE, tm.year + 1'900, tm.mon + 1, tm.mday, tm.hour, tm.min, tm.sec);
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = false;
    }

    return result;
}

bool FS_Service::set_modify_time(FileWrapper& file, const DateTimeFields& tm, file_callback_t callback) const {
    bool result { true };

    auto p_operation { new FileOperation { [&tm](FileWrapper* p_file) {
                                              if constexpr (DEBUG_) {
                                                  Serial.printf(PSTR("SET_MODIFY_TIME operation: p_file=0x%x\r\n"), p_file);
                                              }

                                              if (tm.year < 80 || tm.year > 207) {
                                                  return false;
                                              }

                                              return p_file->fs_file_.timestamp(T_WRITE, tm.year + 1'900, tm.mon + 1, tm.mday, tm.hour, tm.min, tm.sec);
                                          },
        callback ? callback :
                   [&result](FileOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), file } };

    if (!schedule_operation(p_operation, callback ? true : false, file.timeout_ticks_)) {
        result = false;
    }

    return result;
}

File FS_Service::open(SDClass* p_fs, const char* filepath, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper) const {
    File f;
    if constexpr (DEBUG_) {
        Serial.print(PSTR("FS_Service::open()\r\n"));
    }

    auto file { p_fs->sdfs.open(filepath, mode == FILE_READ ? O_READ : (mode == FILE_WRITE ? O_RDWR | O_CREAT | O_AT_END : O_RDWR | O_CREAT)) };
    if (file) {
        auto p_fwrapper { new FileWrapper { *this, file, op_timeout_us } };
        configASSERT(p_fwrapper);
        p_fwrapper->filename_.resize(32);
        file.getName(p_fwrapper->filename_.data(), 32);
        f = p_fwrapper;
        if (p_file_wrapper) {
            *p_file_wrapper = p_fwrapper;
        }
    } else {
        f = File {};
        if (p_file_wrapper) {
            *p_file_wrapper = nullptr;
        }
    }

    return f;
}

File FS_Service::open_next(FsFile& file, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper) const {
    if constexpr (DEBUG_) {
        Serial.print(PSTR("FS_Service::open_next()\r\n"));
    }

    auto f = file.openNextFile(mode == FILE_READ ? O_READ : (mode == FILE_WRITE ? O_RDWR | O_CREAT | O_AT_END : O_RDWR | O_CREAT));
    if (f) {
        auto p_fwrapper { new FileWrapper { *this, f, op_timeout_us } };
        configASSERT(p_fwrapper);
        p_fwrapper->filename_.resize(32);
        f.getName(p_fwrapper->filename_.data(), 32);
        if (p_file_wrapper) {
            *p_file_wrapper = p_fwrapper;
        }
        return File { p_fwrapper };
    }
    if (p_file_wrapper) {
        *p_file_wrapper = nullptr;
    }
    return File {};
}

File FS_Service::open(const char* filepath, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper, fs_callback_t callback) const {
    File f {};

    auto p_operation { new FSOperation { [filepath, mode, op_timeout_us, p_file_wrapper, this](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("OPEN operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return open(p_fs, filepath, mode, op_timeout_us, p_file_wrapper);
                                        },
        callback ? callback :
                   [&f](FSOperation* p_operation) {
                       f = std::get<File>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us));

    return f;
}

bool FS_Service::exists(const char* filepath, uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [filepath](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("EXISTS operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->exists(filepath);
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::mkdir(const char* filepath, uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [filepath](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("MKDIR operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->mkdir(filepath);
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::rename(const char* oldfilepath, const char* newfilepath, uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [oldfilepath, newfilepath](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("RENAME operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->rename(oldfilepath, newfilepath);
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::remove(const char* filepath, uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [filepath](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("REMOVE operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->remove(filepath);
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::rmdir(const char* filepath, uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [filepath](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("RMDIR operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->rmdir(filepath);
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

uint64_t FS_Service::usedSize(uint32_t op_timeout_us, fs_callback_t callback) const {
    uint64_t result {};

    auto p_operation { new FSOperation { [](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("USED_SIZE operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->usedSize();
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<uint64_t>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::format(uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("FORMAT operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->format();
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::mediaPresent(uint32_t op_timeout_us, fs_callback_t callback) const {
    bool result { true };

    auto p_operation { new FSOperation { [](SDClass* p_fs) {
                                            if constexpr (DEBUG_) {
                                                Serial.printf(PSTR("MEDIA_PRESENT operation: p_fs=0x%x\r\n"), p_fs);
                                            }
                                            return p_fs->mediaPresent();
                                        },
        callback ? callback :
                   [&result](FSOperation* p_operation) {
                       result = std::get<bool>(p_operation->result_);
                       ::xTaskNotifyGiveIndexed(p_operation->caller_, TASK_NOTIFY_INDEX_);
                   },
        ::xTaskGetCurrentTaskHandle(), fs_ } };

    if (!schedule_operation(p_operation, callback ? true : false, timeout_to_ticks(op_timeout_us))) {
        result = false;
    }

    return result;
}

bool FS_Service::schedule_operation(queue_t operation, bool custom_callback, TickType_t timeout_ticks) const {
    auto p_operation { new queue_t { operation } };

    if (::xQueueSend(job_queue_, &p_operation, QUEUE_SEND_TIMEOUT_) != pdTRUE) {
        if constexpr (DEBUG_) {
            Serial.printf(PSTR("FS_Service::schedule_operation(): xQueueSend failed, queue_overflows_=%u\r\n"), queue_overflows_);
        }
        return false;
    }
    if constexpr (DEBUG_) {
        Serial.printf(PSTR("FS_Service::schedule_operation(): xQueueSend() done, p_operation=0x%x queue_overflows_=%u\r\n"), p_operation, queue_overflows_);
    }

    if (custom_callback) {
        if constexpr (DEBUG_) {
            Serial.printf(PSTR("FS_Service::schedule_operation(): custom callback set, returning.\r\n"));
        }
        return true;
    }

    if (::ulTaskNotifyTakeIndexed(TASK_NOTIFY_INDEX_, pdTRUE, timeout_ticks) == 0) {
        if (auto p_file_op = std::get_if<FileOperation*>(p_operation)) {
            auto p_file_operation { *p_file_op };
            p_file_operation->canceled_ = true;
            p_file_operation->callback_ = nullptr;
            ::vTaskDelay(1);
        } else if (auto p_fs_op = std::get_if<FSOperation*>(p_operation)) {
            auto p_fs_operation { *p_fs_op };
            p_fs_operation->canceled_ = true;
            p_fs_operation->callback_ = nullptr;
            ::vTaskDelay(1);
        }
        if constexpr (DEBUG_) {
            Serial.printf(PSTR("FS_Service::schedule_operation(): timeout for p_operation=0x%x\r\n"), p_operation);
        }
        return false;
    }

    if constexpr (DEBUG_) {
        Serial.printf(PSTR("FS_Service::schedule_operation(): notify for p_operation=0x%x done.\r\n"), p_operation);
    }
    return true;
}

uint64_t FS_Service::totalSize() const {
    return fs_.totalSize();
}


FS_Service::FileOperation::FileOperation(file_operation_t&& operation, file_callback_t&& callback, TaskHandle_t caller, FileWrapper& file)
    : operation_ { operation }, callback_ { callback }, caller_ { caller }, p_file_ { &file }, canceled_ {} {}

FS_Service::FSOperation::FSOperation(fs_operation_t&& operation, fs_callback_t&& callback, TaskHandle_t caller, SDClass& fs)
    : operation_ { operation }, callback_ { callback }, caller_ { caller }, p_fs_ { &fs }, canceled_ {} {}

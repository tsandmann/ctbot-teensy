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
 * @file    fs_service.h
 * @brief   FreeRTOS service for filesystem on SD card
 * @author  Timo Sandmann
 * @date    19.11.2022
 */

#pragma once

#include "arduino_freertos.h"

#include <string>
#include <functional>
#include <atomic>
#include <variant>
#include <type_traits>


class SDClass;
class SdioConfig;


class FS_Service {
    static constexpr bool DEBUG_ { false };

    static constexpr uint8_t WORKER_TASK_PRIORITY_ { configMAX_PRIORITIES - 2 };
    static constexpr uint32_t WORKER_QUEUE_SIZE_ { 8 };
    static constexpr uint8_t TASK_NOTIFY_INDEX_ { 0 };
    static constexpr uint32_t WORKER_TASK_STACK_SIZE_ { 2048 }; // byte
    static constexpr TickType_t QUEUE_SEND_TIMEOUT_ { portMAX_DELAY };

public:
    class FileWrapper;
    struct FileOperation;
    struct FSOperation;

protected:
    struct dummy_t {};

    using file_result_t = std::variant<bool, int, size_t>;
    using fs_result_t = std::variant<bool, int, size_t, std::conditional_t<std::is_same<uint64_t, size_t>::value, dummy_t, uint64_t>, File>;
    using file_operation_t = std::function<file_result_t(FileWrapper*)>;
    using fs_operation_t = std::function<fs_result_t(SDClass*)>;
    using file_callback_t = std::function<void(FileOperation*)>;
    using fs_callback_t = std::function<void(FSOperation*)>;
    using queue_t = std::variant<FileOperation*, FSOperation*>;

    static inline QueueHandle_t job_queue_ {};
    static inline TaskHandle_t worker_ {};
    static inline uint32_t queue_overflows_ {};

    SDClass& fs_;

    static bool init();

    static void run(void* param);

    static constexpr TickType_t timeout_to_ticks(uint32_t timeout_us) {
        return timeout_us ? pdUS_TO_TICKS(timeout_us) : portMAX_DELAY;
    }

    bool schedule_operation(queue_t operation, bool custom_callback, TickType_t timeout_ticks) const;

    File open(SDClass* p_fs, const char* filepath, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper = nullptr) const;

    File open_next(FsFile& file, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper) const;

    int read(FileWrapper& file, void* buf, size_t size, file_callback_t callback = nullptr) const;

    int peek(FileWrapper& file, file_callback_t callback = nullptr) const;

    int available(FileWrapper& file, file_callback_t callback = nullptr) const;

    size_t write(FileWrapper& file, const void* buf, size_t size, file_callback_t callback = nullptr) const;

    void flush(FileWrapper& file, file_callback_t callback = nullptr) const;

    bool seek(FileWrapper& file, uint64_t pos, int mode = SeekSet, file_callback_t callback = nullptr) const;

    void close(FileWrapper& file, file_callback_t callback = nullptr) const;

    bool truncate(FileWrapper& file, uint64_t size, file_callback_t callback = nullptr) const;

    File open_next_file(FileWrapper& file, uint8_t mode, FileWrapper** p_file_wrapper, file_callback_t callback = nullptr) const;

    void rewind_directory(FileWrapper& file, file_callback_t callback = nullptr) const;

    bool get_create_time(FileWrapper& file, DateTimeFields& tm, file_callback_t callback = nullptr) const;

    bool get_modify_time(FileWrapper& file, DateTimeFields& tm, file_callback_t callback = nullptr) const;

    bool set_create_time(FileWrapper& file, const DateTimeFields& tm, file_callback_t callback = nullptr) const;

    bool set_modify_time(FileWrapper& file, const DateTimeFields& tm, file_callback_t callback = nullptr) const;

public:
    struct FileOperation {
        file_operation_t operation_;
        file_callback_t callback_;
        TaskHandle_t caller_;
        FileWrapper* p_file_;
        std::atomic<bool> canceled_;
        file_result_t result_;

        FileOperation(file_operation_t&& operation, file_callback_t&& callback, TaskHandle_t caller, FileWrapper& file);
    };

    struct FSOperation {
        fs_operation_t operation_;
        fs_callback_t callback_;
        TaskHandle_t caller_;
        SDClass* p_fs_;
        std::atomic<bool> canceled_;
        fs_result_t result_;

        FSOperation(fs_operation_t&& operation, fs_callback_t&& callback, TaskHandle_t caller, SDClass& fs);
    };

    class FileWrapper : public FileImpl {
    protected:
        friend class FS_Service;

        const FS_Service& fs_svc_;
        FsFile fs_file_;
        TickType_t timeout_ticks_;
        std::string filename_;

        FileWrapper(const FS_Service& fs_svc, FsFile& file) : FileWrapper { fs_svc, file, 0 } {}

        FileWrapper(const FS_Service& fs_svc, FsFile& file, uint32_t op_timeout_us)
            : fs_svc_ { fs_svc }, fs_file_ { file }, timeout_ticks_ { op_timeout_us ? pdUS_TO_TICKS(op_timeout_us) : portMAX_DELAY } {}

    public:
        virtual ~FileWrapper() {
            close();
        }

        void set_op_timeout(uint32_t timeout_us) {
            timeout_ticks_ = timeout_us ? pdUS_TO_TICKS(timeout_us) : portMAX_DELAY;
        }

        virtual size_t write(const void* buf, size_t size) override {
            return fs_svc_.write(*this, buf, size);
        }

        virtual size_t write(const void* buf, size_t size, file_callback_t callback) {
            return fs_svc_.write(*this, buf, size, callback);
        }

        virtual int peek() override {
            return fs_svc_.peek(*this);
        }

        virtual int available() override {
            const auto res { fs_svc_.available(*this) };
            return res > 0 ? res : 0;
        }

        virtual void flush() override {
            fs_svc_.flush(*this);
        }

        virtual void flush(file_callback_t callback) {
            fs_svc_.flush(*this, callback);
        }

        virtual size_t read(void* buf, size_t nbyte) override {
            const auto res { fs_svc_.read(*this, buf, nbyte) };
            return res > 0 ? res : 0;
        }

        virtual size_t read(void* buf, size_t nbyte, file_callback_t callback) {
            const auto res { fs_svc_.read(*this, buf, nbyte, callback) };
            return res > 0 ? res : 0;
        }

        virtual bool truncate(uint64_t size = 0) override {
            return fs_svc_.truncate(*this, size);
        }

        virtual bool seek(uint64_t pos, int mode = SeekSet) override {
            return fs_svc_.seek(*this, pos, mode);
        }

        virtual uint64_t position() override {
            return fs_file_.position();
        }

        virtual uint64_t size() override {
            return fs_file_.size();
        }

        virtual void close() override {
            return fs_svc_.close(*this);
        }

        virtual bool isOpen() override {
            return fs_file_;
        }

         virtual const char* name() override {
            return filename_.c_str();
         }

         virtual bool isDirectory() override {
            return fs_file_.isDirectory();
         }

         virtual File openNextFile(uint8_t mode = 0) override {
            return fs_svc_.open_next_file(*this, mode, nullptr, nullptr);
         }

         virtual void rewindDirectory() override {
            fs_svc_.rewind_directory(*this);
         }

         virtual bool getCreateTime(DateTimeFields& tm) override {
            return fs_svc_.get_create_time(*this, tm);
         }

         virtual bool getModifyTime(DateTimeFields& tm) override {
            return fs_svc_.get_modify_time(*this, tm);
         }

         virtual bool setCreateTime(const DateTimeFields& tm) override {
            return fs_svc_.set_create_time(*this, tm);
         }

        virtual bool setModifyTime(const DateTimeFields& tm) override {
            return fs_svc_.set_modify_time(*this, tm);
        }
    };

    FS_Service(SDClass& fs);

    bool begin(SdioConfig&& sdio_config) const;

    File open(const char* filepath) const {
        return open(filepath, FILE_READ, 0, nullptr, nullptr);
    }

    File open(const char* filepath, uint8_t mode) const {
        return open(filepath, mode, 0, nullptr, nullptr);
    }

    File open(const char* filepath, uint8_t mode, uint32_t op_timeout_us) const {
        return open(filepath, mode, op_timeout_us, nullptr, nullptr);
    }

    File open(const char* filepath, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper) const {
        return open(filepath, mode, op_timeout_us, p_file_wrapper, nullptr);
    }

    File open(const char* filepath, uint8_t mode, uint32_t op_timeout_us, FileWrapper** p_file_wrapper, fs_callback_t callback) const;

    bool exists(const char* filepath, uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    bool mkdir(const char* filepath, uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    bool rename(const char* oldfilepath, const char* newfilepath, uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    bool remove(const char* filepath, uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    bool rmdir(const char* filepath, uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    uint64_t usedSize(uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    uint64_t totalSize() const;

    bool format(uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    bool mediaPresent(uint32_t op_timeout_us = 0, fs_callback_t callback = nullptr) const;

    void set_priority(uint8_t new_priority) const;

    void reset_priority() const;
};

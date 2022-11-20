/*
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
 * @file    SD.h
 * @brief   Wrapper aroung Arduino SD library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#pragma once

#include "Arduino.h"
#include "FS.h"

#include <string>
#include <string_view>
#include <cstdint>
#include <cstdio>
#include <cassert>


// static constexpr uint8_t O_READ { 0X01 };
// static constexpr uint8_t O_RDONLY { O_READ };
// static constexpr uint8_t O_WRITE { 0X02 };
// static constexpr uint8_t O_WRONLY { O_WRITE };
// static constexpr uint8_t O_RDWR { O_READ | O_WRITE };
// static constexpr uint8_t O_ACCMODE { O_READ | O_WRITE };
// static constexpr uint8_t O_APPEND { 0X04 };
// static constexpr uint8_t O_SYNC { 0X08 };
// static constexpr uint8_t O_CREAT { 0X10 };
// static constexpr uint8_t O_EXCL { 0X20 };
// static constexpr uint8_t O_TRUNC { 0X40 };

#ifndef FIFO_SDIO
static constexpr uint8_t FIFO_SDIO { 0 };
#endif

#ifndef DMA_SDIO
static constexpr uint8_t DMA_SDIO { 1 };
#endif

// FIXME: untested!
class FileImplStdio : public FileImpl {
    FILE* p_file_;
    const int mode_;
    const std::string name_;

    friend class SDClass;
    friend class FsBaseFile;

public:
    FileImplStdio();
    FileImplStdio(const std::string_view& file, const int mode);

    virtual ~FileImplStdio();

    virtual size_t read(void* buf, size_t nbyte) override;
    virtual size_t write(const void* buf, size_t size) override;
    virtual int available() override;
    virtual int peek() override;
    virtual void flush() override;
    virtual bool truncate(uint64_t size = 0) override;
    virtual bool seek(uint64_t pos, int mode) override;
    virtual uint64_t position() override;
    virtual uint64_t size() override;
    virtual void close() override;
    virtual bool isOpen() override {
        return p_file_ != nullptr;
    }
    virtual const char* name() override {
        return name_.c_str();
    }
    virtual bool isDirectory() override;
    virtual File openNextFile([[maybe_unused]] uint8_t mode = 0) override {
        assert(false);
        return File {};
    }
    virtual void rewindDirectory() override {}
    virtual bool getCreateTime(DateTimeFields&) override {
        return false;
    }
    virtual bool getModifyTime(DateTimeFields&) override {
        return false;
    }
    virtual bool setCreateTime(const DateTimeFields&) override {
        return false;
    }
    virtual bool setModifyTime(const DateTimeFields&) override {
        return false;
    }
};


class SdioConfig {
public:
    SdioConfig() {}

    explicit SdioConfig(uint8_t) {}

    uint8_t options() const {
        return 0;
    }

    bool useDma() const {
        return false;
    }
};


class SDClass : public FsVolume {
public:
    SDClass() : sdfs { *this } {}

    bool begin(uint8_t = 0) {
        return true;
    }

    bool begin(const SdioConfig&) {
        return true;
    }

    virtual FsFile open(const char* path, oflag_t oflag = O_RDONLY);
    virtual bool exists(const char* filepath);
    virtual bool mkdir(const char* filepath);
    virtual bool rename(const char* oldfilepath, const char* newfilepath);
    virtual bool remove(const char* filepath);
    virtual bool rmdir(const char* filepath);
    virtual uint64_t usedSize();
    virtual uint64_t totalSize();

    virtual bool mediaPresent() {
        return true;
    }

    virtual bool format(int = 0, char = 0, Print& = Serial) {
        return false;
    }

    SDClass& sdfs;
};

extern SDClass SD;

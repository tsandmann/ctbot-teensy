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
 * @file    SD.cpp
 * @brief   Wrapper aroung Arduino SD library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#include "SD.h"

#include <filesystem>
#include <limits>


FileImplStdio::FileImplStdio() : p_file_ {}, mode_ {} {}

FileImplStdio::FileImplStdio(const std::string_view& file, const int mode) : p_file_ {}, mode_ { mode }, name_ { file } {
    std::string filemode { "rb" };
    if (mode & O_RDWR) {
        filemode = "a+b";
    }
    p_file_ = std::fopen(name_.data(), filemode.c_str());
    if (p_file_) {
        if (!(mode & O_AT_END)) {
            std::rewind(p_file_);
        }
    }
}

FileImplStdio::~FileImplStdio() {
    if (p_file_) {
        std::fclose(p_file_);
    }
}

size_t FileImplStdio::write(const void* buf, size_t nbyte) {
    if (!p_file_) {
        return 0;
    }
    const auto res { std::fwrite(buf, 1, nbyte, p_file_) };
    flush();
    return res;
}

size_t FileImplStdio::read(void* buf, size_t nbyte) {
    if (!p_file_) {
        return 0;
    }
    return std::fread(buf, 1, nbyte, p_file_);
}

int FileImplStdio::peek() {
    if (!p_file_) {
        return -1;
    }
    const auto value { std::fgetc(p_file_) };
    std::ungetc(value, p_file_);

    return value;
}

int FileImplStdio::available() {
    return size() - position();
}

uint64_t FileImplStdio::position() {
    if (!p_file_) {
        return 0;
    }
    return std::ftell(p_file_);
}

void FileImplStdio::flush() {
    if (!p_file_) {
        return;
    }
    std::fflush(p_file_);
}

bool FileImplStdio::truncate(uint64_t size) {
    if (!p_file_) {
        return false;
    }
    if (!(mode_ & O_RDWR)) {
        return false;
    }

    fpos_t pos;
    std::fgetpos(p_file_, &pos);
    close();

    std::error_code e;
    std::filesystem::resize_file(name_, size, e);
    if (e.value()) {
        return false;
    }
    p_file_ = std::fopen(name_.data(), "a+b");
    std::fsetpos(p_file_, &pos);

    return true;
}

bool FileImplStdio::seek(uint64_t pos, int mode) {
    if (!p_file_) {
        return false;
    }
    return std::fseek(p_file_, pos, mode == SeekSet ? SEEK_SET : (mode == SeekEnd ? SEEK_END : SEEK_CUR)) == 0;
}

uint64_t FileImplStdio::size() {
    std::error_code e;
    return std::filesystem::file_size(name_, e);
}

void FileImplStdio::close() {
    if (p_file_) {
        std::fclose(p_file_);
        p_file_ = nullptr;
    }
}


bool FileImplStdio::isDirectory() {
    std::error_code e;
    return std::filesystem::is_directory(name_, e);
}


FsFile SDClass::open(const char* path, oflag_t oflag) {
    FsFile tmpFile;
    tmpFile.open(this, path, oflag);
    return tmpFile;
}

bool SDClass::exists(const char* name) {
    std::error_code e;
    return std::filesystem::exists(name, e);
}

bool SDClass::remove(const char* name) {
    std::error_code e;
    return std::filesystem::remove(name, e);
}

bool SDClass::mkdir(const char* filepath) {
    std::error_code e;
    return std::filesystem::create_directory(filepath, e);
}

bool SDClass::rename(const char* oldfilepath, const char* newfilepath) {
    std::error_code e;
    std::filesystem::rename(oldfilepath, newfilepath, e);
    return e.value() == 0;
}

bool SDClass::rmdir(const char* filepath) {
    std::error_code e;
    return std::filesystem::remove_all(filepath, e) > 0;
}

uint64_t SDClass::usedSize() {
    std::error_code e;
    auto info { std::filesystem::space(".", e) };
    return info.capacity - info.available;
}

uint64_t SDClass::totalSize() {
    std::error_code e;
    auto info { std::filesystem::space(".", e) };
    return info.capacity;
}

SDClass SD;


FsBaseFile::FsBaseFile(const FsBaseFile& from) : p_file_ {} {
    if (from.p_file_) {
        char buf[33];
        buf[sizeof(buf) - 1] = 0;
        from.getName(buf, sizeof(buf) - 1);
        open(buf, from.mode_);
    }
}

FsBaseFile& FsBaseFile::operator=(const FsBaseFile& from) {
    if (this == &from) {
        return *this;
    }
    close();

    if (from.p_file_) {
        char buf[33];
        buf[sizeof(buf) - 1] = 0;
        from.getName(buf, sizeof(buf) - 1);
        open(buf, from.mode_);
    }
    return *this;
}

bool FsBaseFile::open(const char* path, oflag_t oflag) {
    p_file_ = new FileImplStdio { path, oflag };
    mode_ = oflag;
    return p_file_ != nullptr;
}

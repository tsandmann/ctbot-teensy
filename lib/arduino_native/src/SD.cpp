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


FileImplStdio::FileImplStdio(const std::string_view& file, const uint8_t mode) : p_file_ {}, mode_ { mode }, name_ { file } {
    std::string filemode { "rb" };
    if (mode & FILE_WRITE) {
        filemode = "a+b";
    }
    p_file_ = std::fopen(name_.data(), filemode.c_str());
    if (mode & FILE_WRITE_BEGIN) {
        std::rewind(p_file_);
    }
}

FileImplStdio::~FileImplStdio() {
    if (p_file_) {
        std::fclose(p_file_);
    }
}

size_t FileImplStdio::write(const void* buf, size_t nbyte) {
    return std::fwrite(buf, 1, nbyte, p_file_);
}

size_t FileImplStdio::read(void* buf, size_t nbyte) {
    return std::fread(buf, 1, nbyte, p_file_);
}

int FileImplStdio::peek() {
    const auto value { std::fgetc(p_file_) };
    std::ungetc(value, p_file_);

    return value;
}

int FileImplStdio::available() {
    return size() - position();
}

uint64_t FileImplStdio::position() {
    return std::ftell(p_file_);
}

void FileImplStdio::flush() {
    std::fflush(p_file_);
}

bool FileImplStdio::truncate(uint64_t size) {
    if (mode_ == FILE_READ) {
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
    return std::fseek(p_file_, pos, mode == SeekSet ? SEEK_SET : (mode == SeekEnd ? SEEK_END : SEEK_CUR)) == 0;
}

uint64_t FileImplStdio::size() {
    std::error_code e;
    return std::filesystem::file_size(name_, e);
}

void FileImplStdio::close() {
    std::fclose(p_file_);
    p_file_ = nullptr;
}


bool FileImplStdio::isDirectory() {
    std::error_code e;
    return std::filesystem::is_directory(name_, e);
}


File SDClass::open(const char* filepath, uint8_t mode) {
    return File { new FileImplStdio { filepath, mode } };
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

/* Teensyduino Core Library - File base class
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2021 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file    FS.h
 * @brief   Wrapper aroung Arduino FS library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    19.11.2022
 * @note    Many parts copied from Teensyduino FS.h
 */

#pragma once

#include "Arduino.h"

#define FILE_READ 0
#define FILE_WRITE 1
#define FILE_WRITE_BEGIN 2

#define O_RDONLY 0 /* +1 == FREAD */
#define O_WRONLY 1 /* +1 == FWRITE */
#define O_RDWR 2 /* +1 == FREAD|FWRITE */
#define O_CREAT 0x0200 // _FCREAT
#define O_TRUNC _FTRUNC
#define O_EXCL _FEXCL
#define O_SYNC _FSYNC
#define O_AT_END 0x4000 // _FNONBLOCK

#define O_READ O_RDONLY
#define O_WRITE O_WRONLY

enum SeekMode { SeekSet = 0, SeekCur = 1, SeekEnd = 2 };

typedef int oflag_t;
typedef Print print_t;
typedef Stream stream_t;
typedef uint32_t newalign_t;

typedef struct {
    uint64_t position;
    uint32_t cluster;
} fspos_t;

class File;
class FsFile;
class FileImplStdio;


class FsVolume {};


class FileImpl {
public:
    virtual ~FileImpl() {}
    virtual size_t read(void* buf, size_t nbyte) = 0;
    virtual size_t write(const void* buf, size_t size) = 0;
    virtual int available() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
    virtual bool truncate(uint64_t size = 0) = 0;
    virtual bool seek(uint64_t pos, int mode) = 0;
    virtual uint64_t position() = 0;
    virtual uint64_t size() = 0;
    virtual void close() = 0;
    virtual bool isOpen() = 0;
    virtual const char* name() = 0;
    virtual bool isDirectory() = 0;
    virtual File openNextFile(uint8_t mode = 0) = 0;
    virtual void rewindDirectory(void) = 0;
    virtual bool getCreateTime(DateTimeFields&) {
        return false;
    }
    virtual bool getModifyTime(DateTimeFields&) {
        return false;
    }
    virtual bool setCreateTime(const DateTimeFields&) {
        return false;
    }
    virtual bool setModifyTime(const DateTimeFields&) {
        return false;
    }

private:
    friend class File;
    unsigned int refcount = 0; // number of File instances referencing this FileImpl
};


class File final : public Stream {
public:
    // Empty constructor, used when a program creates a File variable
    // but does not immediately assign or initialize it.
    constexpr File() : f(nullptr) {}

    // Explicit FileImpl constructor.  Used by libraries which provide
    // access to files stored on media.  Normally this is used within
    // functions derived from FS::open() and FileImpl::openNextFile().
    // Not normally called used from ordinary programs or libraries
    // which only access files.
    File(FileImpl* file) {
        f = file;
        if (f)
            f->refcount++;
        // Serial.printf("File ctor %x, refcount=%d\n", (int)f, get_refcount());
    }

    // Copy constructor.  Typically used when a File is passed by value
    // into a function.  The File instance within the called function is
    // a copy of the original.  Also used when a File instance is created
    // and assigned a value (eg, "File f =
    File(const File& file) {
        f = file.f;
        if (f)
            f->refcount++;
        // Serial.printf("File copy ctor %x, refcount=%d\n", (int)f, get_refcount());
    }
#ifdef FILE_USE_MOVE
    // Move constructor.
    File(const File&& file) {
        f = file.f;
        if (f)
            f->refcount++;
        // Serial.printf("File copy ctor %x, refcount=%d\n", (int)f, get_refcount());
    }
#endif
    // Copy assignment.
    File& operator=(const File& file) {
        // Serial.println("File copy assignment");
        if (file.f)
            file.f->refcount++;
        if (f) {
            dec_refcount(); /*Serial.println("File copy assignment autoclose");*/
        }
        f = file.f;
        return *this;
    }
#ifdef FILE_USE_MOVE
    // Move assignment.
    File& operator=(const File&& file) {
        // Serial.println("File move assignment");
        if (file.f)
            file.f->refcount++;
        if (f) {
            dec_refcount(); /*Serial.println("File move assignment autoclose");*/
        }
        f = file.f;
        return *this;
    }
#endif
    virtual ~File() {
        // Serial.printf("File dtor %x, refcount=%d\n", (int)f, get_refcount());
        if (f)
            dec_refcount();
    }
    size_t read(void* buf, size_t nbyte) {
        return (f) ? f->read(buf, nbyte) : 0;
    }
    size_t write(const void* buf, size_t size) {
        return (f) ? f->write(buf, size) : 0;
    }
    int available() {
        return (f) ? f->available() : 0;
    }
    int peek() {
        return (f) ? f->peek() : -1;
    }
    void flush() {
        if (f)
            f->flush();
    }
    bool truncate(uint64_t size = 0) {
        return (f) ? f->truncate(size) : false;
    }
    bool seek(uint64_t pos, int mode) {
        return (f) ? f->seek(pos, mode) : false;
    }
    uint64_t position() {
        return (f) ? f->position() : 0;
    }
    uint64_t size() {
        return (f) ? f->size() : 0;
    }
    void close() {
        if (f) {
            f->close();
            dec_refcount();
        }
    }
    operator bool() {
        return (f) ? f->isOpen() : false;
    }
    const char* name() {
        return (f) ? f->name() : "";
    }
    bool isDirectory() {
        return (f) ? f->isDirectory() : false;
    }
    File openNextFile(uint8_t mode = 0) {
        return (f) ? f->openNextFile(mode) : *this;
    }
    void rewindDirectory(void) {
        if (f)
            f->rewindDirectory();
    }
    bool getCreateTime(DateTimeFields& tm) {
        return (f) ? f->getCreateTime(tm) : false;
    }
    bool getModifyTime(DateTimeFields& tm) {
        return (f) ? f->getModifyTime(tm) : false;
    }
    bool setCreateTime(const DateTimeFields& tm) {
        return (f) ? f->setCreateTime(tm) : false;
    }
    bool setModifyTime(const DateTimeFields& tm) {
        return (f) ? f->setModifyTime(tm) : false;
    }
    bool seek(uint64_t pos) {
        return seek(pos, SeekSet);
    }
    int read() {
        if (!f)
            return -1;
        unsigned char b;
        if (f->read(&b, 1) < 1)
            return -1;
        return b;
    }
    size_t write(uint8_t b) {
        return (f) ? f->write(&b, 1) : 0;
    }
    size_t write(const char* str) {
        return (f) ? f->write(str, strlen(str)) : 0;
    }
    size_t readBytes(char* buffer, size_t length) {
        return read(buffer, length);
    }
    size_t write(unsigned long n) {
        return write((uint8_t) n);
    }
    size_t write(long n) {
        return write((uint8_t) n);
    }
    size_t write(unsigned int n) {
        return write((uint8_t) n);
    }
    size_t write(int n) {
        return write((uint8_t) n);
    }
    using Print::write;

private:
    void dec_refcount() {
        if (--(f->refcount) == 0) {
            f->close();
            delete f;
        }
        f = nullptr;
    }
    int get_refcount() {
        if (f == nullptr)
            return -1;
        return f->refcount;
    }
    FileImpl* f;
};


class FS {
public:
    FS() {}
    virtual bool exists(const char* filepath) = 0;
    virtual bool mkdir(const char* filepath) = 0;
    virtual bool rename(const char* oldfilepath, const char* newfilepath) = 0;
    virtual bool remove(const char* filepath) = 0;
    virtual bool rmdir(const char* filepath) = 0;
    virtual uint64_t usedSize() = 0;
    virtual uint64_t totalSize() = 0;
};

template <class BaseFile, typename PosType>
class StreamFile : public stream_t, public BaseFile {
public:
    using BaseFile::clearWriteError;
    using BaseFile::getWriteError;
    using BaseFile::read;
    using BaseFile::write;

    StreamFile() {}

    int available() {
        return BaseFile::available();
    }

    void flush() {
        BaseFile::sync();
    }

    bool isDirectory() {
        return BaseFile::isDir();
    }

    int peek() {
        return BaseFile::peek();
    }

    PosType position() {
        return BaseFile::curPosition();
    }

    int read() {
        return BaseFile::read();
    }

    void rewindDirectory() {
        if (BaseFile::isDir()) {
            BaseFile::rewind();
        }
    }

    bool seek(PosType pos) {
        return BaseFile::seekSet(pos);
    }

    PosType size() {
        return BaseFile::fileSize();
    }

    size_t write(uint8_t b) {
        return BaseFile::write(b);
    }

    size_t write(const uint8_t* buffer, size_t size) {
        return BaseFile::write(buffer, size);
    }
};

class FsBaseFile {
public:
    FsBaseFile() : p_file_ {}, mode_ {} {}

    FsBaseFile(const char* path, oflag_t oflag) : p_file_ {}, mode_ { oflag } {
        open(path, oflag);
    }

    FsBaseFile(const FsBaseFile& from);

    FsBaseFile& operator=(const FsBaseFile& from);

    ~FsBaseFile() {
        close();
    }

    operator bool() const {
        return isOpen();
    }

    int available() const {
        if (p_file_) {
            return p_file_->available();
        }
        return 0;
    }

    uint64_t available64() const {
        if (p_file_) {
            return p_file_->available();
        }
        return 0;
    }

    void clearWriteError() {}

    bool close() {
        if (p_file_) {
            p_file_->close();
            delete p_file_;
            p_file_ = nullptr;
            return true;
        }
        return false;
    }

    // bool contiguousRange(uint32_t*, uint32_t*) {
    //     return false;
    // }

    uint64_t curPosition() const {
        if (p_file_) {
            return p_file_->position();
        }
        return 0;
    }

    // uint32_t dirIndex() const {
    //     return 0;
    // }

    // bool exists(const char*) {
    //     return false;
    // }

    // void fgetpos(fspos_t*) const {}

    // int fgets(char*, int, char* = nullptr) {
    //     return -1;
    // }

    uint64_t fileSize() const {
        if (p_file_) {
            return p_file_->size();
        }
        return 0;
    }

    // uint32_t firstSector() const {
    //     return 0;
    // }

    void flush() {
        sync();
    }

    // void fsetpos(const fspos_t*) {}

    // bool getAccessDateTime(uint16_t*, uint16_t*) {
    //     return false;
    // }

    bool getCreateDateTime(uint16_t*, uint16_t*) {
        return false;
    }

    // uint8_t getError() const {
    //     return 0XFF;
    // }

    bool getModifyDateTime(uint16_t*, uint16_t*) {
        return false;
    }

    size_t getName(char* name, size_t len) const {
        if (!p_file_) {
            return 0;
        }
        std::strncpy(name, p_file_->name(), len);
        return std::strlen(p_file_->name());
    }

    bool getWriteError() const {
        return false;
    }

    bool isBusy() {
        return false;
    }

    bool isContiguous() const {
        return false;
    }

    bool isDir() const {
        return false;
    }

    bool isDirectory() const {
        return isDir();
    }

    bool isFile() const {
        return true;
    }

    bool isHidden() const {
        return false;
    }

    bool isOpen() const {
        return p_file_ ? p_file_->isOpen() : false;
    }

    bool isReadable() const {
        return p_file_;
    }

    bool isReadOnly() const {
        return mode_ == O_READ;
    }

    bool isSubDir() const {
        return false;
    }

    bool isWritable() const {
        return mode_ & O_RDWR;
    }

    // bool ls(uint8_t flags) {
    //     return ls(&Serial, flags);
    // }

    // bool ls() {
    //     return ls(&Serial);
    // }

    // bool ls(print_t*) {
    //     return false;
    // }

    // bool ls(print_t*, uint8_t) {
    //     return false;
    // }

    bool mkdir(FsBaseFile* dir, const char* path, bool pFlag = true);

    bool open(FsBaseFile* dir, const char* path, oflag_t oflag = O_RDONLY);

    bool open(FsBaseFile* dir, uint32_t index, oflag_t oflag);

    bool open(FsVolume*, const char* path, oflag_t oflag) {
        return open(path, oflag);
    }

    bool open(const char* path, oflag_t oflag = O_RDONLY);

    bool openNext(FsBaseFile*, oflag_t = O_RDONLY) {
        return false; // FIXME: implement
    }

    bool openRoot(FsVolume* vol);

    uint64_t position() const {
        return curPosition();
    }

    int peek() {
        if (p_file_) {
            return p_file_->peek();
        }
        return -1;
    }

    // bool preAllocate(uint64_t) {
    //     return false;
    // }

    // size_t printAccessDateTime(print_t*) {
    //     return 0;
    // }

    // size_t printCreateDateTime(print_t*) {
    //     return 0;
    // }

    // size_t printField(double, char, uint8_t = 2) {
    //     return 0;
    // }

    // size_t printField(float value, char term, uint8_t prec = 2) {
    //     return printField(static_cast<double>(value), term, prec);
    // }

    // template <typename Type>
    // size_t printField(Type, char) {
    //     return 0;
    // }

    // size_t printFileSize(print_t*) {
    //     return 0;
    // }

    // size_t printModifyDateTime(print_t*) {
    //     return 0;
    // }

    // size_t printName(print_t*) {
    //     return 0;
    // }

    int read() {
        uint8_t b;
        return read(&b, 1) == 1 ? b : -1;
    }

    int read(void* buf, size_t length) {
        if (p_file_) {
            const auto res { p_file_->read(buf, length) };
            return res > 0 ? res : -1;
        }
        return -1;
    }

    // bool remove();

    // bool remove(const char*) {
    //     return false;
    // }

    // bool rename(const char*) {
    //     return false;
    // }

    // bool rename(FsBaseFile*, const char*) {
    //     return false;
    // }

    void rewind() {
        seekSet(0);
    }

    void rewindDirectory() {
        if (p_file_) {
            p_file_->rewindDirectory();
        }
    }

    // bool rmdir();

    bool seek(uint64_t pos) {
        return seekSet(pos);
    }

    bool seekCur(int64_t offset) {
        return seekSet(curPosition() + offset);
    }

    bool seekEnd(int64_t offset = 0) {
        return seekSet(fileSize() + offset);
    }

    bool seekSet(uint64_t) {
        return false;
    }

    uint64_t size() const {
        return fileSize();
    }

    bool sync() {
        if (p_file_) {
            p_file_->flush();
            return true;
        }
        return false;
    }

    bool timestamp(uint8_t, uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t) {
        return false;
    }

    bool truncate() {
        if (p_file_) {
            return p_file_->truncate();
        }
        return false;
    }

    bool truncate(uint64_t size) {
        if (p_file_) {
            return p_file_->truncate(size);
        }
        return false;
    }

    size_t write(const char* str) {
        return write(str, strlen(str));
    }

    size_t write(uint8_t b) {
        return write(&b, 1);
    }

    size_t write(const void* buf, size_t length) {
        if (p_file_) {
            return p_file_->write(buf, length);
        }
        return 0;
    }

protected:
    FileImpl* p_file_;
    int mode_;
};

class FsFile : public StreamFile<FsBaseFile, uint64_t> {
public:
    FsFile openNextFile(oflag_t oflag = O_RDONLY) {
        FsFile tmpFile;
        tmpFile.openNext(this, oflag);
        return tmpFile;
    }
};

static inline uint16_t FS_DATE(uint16_t year, uint8_t month, uint8_t day) {
    year -= 1980;
    return year > 127 || month > 12 || day > 31 ? 0 : year << 9 | month << 5 | day;
}

static inline uint16_t FS_YEAR(uint16_t fatDate) {
    return 1980 + (fatDate >> 9);
}

static inline uint8_t FS_MONTH(uint16_t fatDate) {
    return (fatDate >> 5) & 0XF;
}

static inline uint8_t FS_DAY(uint16_t fatDate) {
    return fatDate & 0X1F;
}

static inline uint16_t FS_TIME(uint8_t hour, uint8_t minute, uint8_t second) {
    return hour > 23 || minute > 59 || second > 59 ? 0 : hour << 11 | minute << 5 | second >> 1;
}

static inline uint8_t FS_HOUR(uint16_t fatTime) {
    return fatTime >> 11;
}

static inline uint8_t FS_MINUTE(uint16_t fatTime) {
    return (fatTime >> 5) & 0X3F;
}

static inline uint8_t FS_SECOND(uint16_t fatTime) {
    return 2 * (fatTime & 0X1F);
}

static const uint8_t T_ACCESS = 1;
static const uint8_t T_CREATE = 2;
static const uint8_t T_WRITE = 4;

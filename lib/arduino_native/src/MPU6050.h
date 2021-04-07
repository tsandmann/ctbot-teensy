#pragma once

#include "helper_3dmath.h"

#include <cstdint>


class MPU6050 {
public:
    MPU6050(uint8_t = 0) {}

    void initialize() {}

    bool testConnection() {
        return false;
    }

    uint8_t dmpInitialize() {
        return 1;
    }

    void setXAccelOffset(int16_t) {}
    void setYAccelOffset(int16_t) {}
    void setZAccelOffset(int16_t) {}
    void setXGyroOffset(int16_t) {}
    void setYGyroOffset(int16_t) {}
    void setZGyroOffset(int16_t) {}

    void CalibrateGyro(uint8_t = 15) {}
    void CalibrateAccel(uint8_t = 15) {}

    void PrintActiveOffsets() {}

    void setDMPEnabled(bool) {}

    void resetFIFO() {}

    uint8_t dmpGetCurrentFIFOPacket(uint8_t*) {
        return 1;
    }

    uint8_t dmpGetQuaternion(Quaternion*, const uint8_t* = nullptr) {
        return 1;
    }

    uint8_t dmpGetGyro(VectorInt16*, const uint8_t* = nullptr) {
        return 1;
    }

    uint8_t dmpGetAccel(VectorInt16*, const uint8_t* = nullptr) {
        return 1;
    }

    uint8_t dmpGetEuler(float*, Quaternion*) {
        return 1;
    }

    uint8_t dmpGetGravity(VectorFloat*, Quaternion*) {
        return 1;
    }

    uint8_t dmpGetYawPitchRoll(float*, Quaternion*, VectorFloat*) {
        return 1;
    }

    uint8_t dmpGetLinearAccel(VectorInt16*, VectorInt16*, VectorFloat*) {
        return 1;
    }

    uint8_t dmpGetLinearAccelInWorld(VectorInt16*, VectorInt16*, Quaternion*) {
        return 1;
    }
};

class I2Cdev {
public:
    static bool init(const uint8_t, const uint32_t) {
        return false;
    }
};

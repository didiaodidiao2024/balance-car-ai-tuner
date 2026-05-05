#include "mpu6050.h"
#include <Wire.h>

static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool MPU6050::begin() {
    Wire.begin(21, 22);
    Wire.setClock(400000);
    Wire.setTimeOut(10);  // 10ms I2C timeout，防止卡死触发WDT

    writeReg(ADDR, 0x6B, 0x00);
    delay(10);
    writeReg(ADDR, 0x1B, 0x00);
    writeReg(ADDR, 0x1C, 0x00);
    writeReg(ADDR, 0x1A, 0x03);

    Wire.beginTransmission(ADDR);
    Wire.write(0x75);
    if (Wire.endTransmission(false) != 0) return false;  // no ACK = not connected
    Wire.requestFrom((uint8_t)ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t who = Wire.read();
    return (who == 0x68);
}

bool MPU6050::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    uint8_t n = Wire.requestFrom((uint8_t)ADDR, (uint8_t)14);
    if (n < 14) {
        // I2C 读取不完整（震动/接触不良），清空缓冲区并返回失败
        while (Wire.available()) Wire.read();
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // temp
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
    return true;
}

void MPU6050::update(float dt) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(ax, ay, az, gx, gy, gz)) {
        // I2C失败：累积失败时间，主循环据此决定是否停机
        imu_fail_ms += dt * 1000.0f;
        return;
    }
    // 读取成功，重置失败计时
    imu_fail_ms = 0.0f;

    ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

    // Accelerometer angle (pitch around Y axis)
    float ax_f = ax / 16384.0f;
    float ay_f = ay / 16384.0f;
    float az_f = az / 16384.0f;
    float accel_pitch = atan2f(ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * 180.0f / M_PI;
    float accel_roll  = atan2f(ay_f, az_f) * 180.0f / M_PI;

    // Gyroscope rate (°/s)
    float gyro_pitch_rate = gy / 131.0f;
    float gyro_roll_rate  = gx / 131.0f;

    // Complementary filter
    _pitch = ALPHA * (_pitch + gyro_pitch_rate * dt) + (1.0f - ALPHA) * accel_pitch;
    _roll  = ALPHA * (_roll  + gyro_roll_rate  * dt) + (1.0f - ALPHA) * accel_roll;
}

void MPU6050::calibrate(int samples) {
    long sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
    int16_t ax, ay, az, gx, gy, gz;
    for (int i = 0; i < samples; i++) {
        readRaw(ax, ay, az, gx, gy, gz);
        sax += ax; say += ay; saz += az;
        sgx += gx; sgy += gy; sgz += gz;
        delay(2);
    }
    ax_offset = sax / samples;
    ay_offset = say / samples;
    az_offset = saz / samples - 16384;  // remove 1g from Z
    gx_offset = sgx / samples;
    gy_offset = sgy / samples;
    gz_offset = sgz / samples;
}

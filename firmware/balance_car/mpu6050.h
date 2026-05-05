#pragma once
#include <Arduino.h>

class MPU6050 {
public:
    bool begin();
    void update(float dt);   // call every control loop tick
    float getPitch() const { return _pitch; }
    float getRoll()  const { return _roll; }
    // I2C连续失败时间（ms），主循环检查此值决定是否停机
    float imu_fail_ms = 0.0f;

    // Raw calibration offsets (set after calibration)
    int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

    void calibrate(int samples = 500);

private:
    float _pitch = 0, _roll = 0;
    static constexpr float ALPHA = 0.98f;  // complementary filter weight
    static constexpr uint8_t ADDR = 0x68;

    bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz);
};

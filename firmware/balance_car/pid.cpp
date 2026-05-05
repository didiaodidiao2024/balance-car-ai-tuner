#include "pid.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float out_min, float out_max)
    : kp(kp), ki(ki), kd(kd), setpoint(0),
      output_min(out_min), output_max(out_max),
      _integral(0), _prev_error(0), _d_filtered(0) {}

float PID::compute(float input, float dt) {
    float error = setpoint - input;

    // 只在 ki 有效时才积分，ki=0 时不累积
    // 限制 ki*integral 不超过输出范围的 50%（防止 ki 极小时积分无限累积）
    if (ki != 0.0f) {
        _integral += error * dt;
        float half_range = (output_max - output_min) * 0.5f;
        if (ki * _integral >  half_range) _integral =  half_range / ki;
        if (ki * _integral < -half_range) _integral = -half_range / ki;
    }

    // D 项低通滤波（alpha=0.15），抑制 200Hz 下 MPU6050 噪声放大
    float d_raw = (dt > 0) ? (error - _prev_error) / dt : 0;
    _d_filtered = 0.15f * d_raw + 0.85f * _d_filtered;
    _prev_error = error;

    float output = kp * error + ki * _integral + kd * _d_filtered;
    return constrain(output, output_min, output_max);
}

void PID::reset() {
    _integral = 0;
    _prev_error = 0;
    _d_filtered = 0;
}

void PID::setGains(float p, float i, float d) {
    kp = p; ki = i; kd = d;
    reset();
}

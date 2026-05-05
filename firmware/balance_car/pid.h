#pragma once

class PID {
public:
    float kp, ki, kd;
    float setpoint;
    float output_min, output_max;

    PID(float kp, float ki, float kd, float out_min, float out_max);
    float compute(float input, float dt);
    void reset();
    void setGains(float kp, float ki, float kd);

private:
    float _integral;
    float _prev_error;
    float _d_filtered;
};

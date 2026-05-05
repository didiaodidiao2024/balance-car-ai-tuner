#pragma once
#include <Arduino.h>

// Encoder pins
#define ENC_L_A  32
#define ENC_L_B  33
#define ENC_R_A  16
#define ENC_R_B  17

// Physical constants
// 21.3 gear ratio, 11 PPR base, quadrature x4 = 44 PPR motor shaft
// Wheel shaft PPR = 44 * 21.3 = 936.2
// Wheel circumference = PI * 68mm = 213.6mm
static constexpr float WHEEL_CIRC_MM  = 213.628f;
static constexpr float PULSES_PER_REV = 936.2f;
static constexpr float MM_PER_PULSE   = WHEEL_CIRC_MM / PULSES_PER_REV;

void encoderInit();

// Call every dt seconds to get speed in m/s
float encoderSpeedLeft(float dt);
float encoderSpeedRight(float dt);

// Raw pulse counts (for debugging)
extern volatile int32_t enc_count_l;
extern volatile int32_t enc_count_r;

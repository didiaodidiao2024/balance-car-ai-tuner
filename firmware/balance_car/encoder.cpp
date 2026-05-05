#include "encoder.h"
#include <soc/gpio_struct.h>

volatile int32_t enc_count_l = 0;
volatile int32_t enc_count_r = 0;

static int32_t last_count_l = 0;
static int32_t last_count_r = 0;

// 直接读GPIO寄存器，比digitalRead快约10倍
#define FAST_READ(pin) (((pin) < 32) ? ((GPIO.in >> (pin)) & 1u) \
                                     : ((GPIO.in1.val >> ((pin) - 32)) & 1u))

// Left encoder ISRs
static void IRAM_ATTR isr_l_a() {
    if (FAST_READ(ENC_L_A) == FAST_READ(ENC_L_B)) enc_count_l++;
    else enc_count_l--;
}
static void IRAM_ATTR isr_l_b() {
    if (FAST_READ(ENC_L_A) != FAST_READ(ENC_L_B)) enc_count_l++;
    else enc_count_l--;
}

// Right encoder ISRs
static void IRAM_ATTR isr_r_a() {
    if (FAST_READ(ENC_R_A) == FAST_READ(ENC_R_B)) enc_count_r++;
    else enc_count_r--;
}
static void IRAM_ATTR isr_r_b() {
    if (FAST_READ(ENC_R_A) != FAST_READ(ENC_R_B)) enc_count_r++;
    else enc_count_r--;
}

void encoderInit() {
    // GPIO32/33 支持内部上拉，用 INPUT_PULLUP 避免悬空噪声触发假中断
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    // GPIO16/17 支持内部上拉
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_l_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_l_b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_r_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_r_b, CHANGE);
}

float encoderSpeedLeft(float dt) {
    int32_t cur = enc_count_l;
    int32_t delta = cur - last_count_l;
    last_count_l = cur;
    return (delta * MM_PER_PULSE) / (dt * 1000.0f);  // m/s
}

float encoderSpeedRight(float dt) {
    int32_t cur = enc_count_r;
    int32_t delta = cur - last_count_r;
    last_count_r = cur;
    return (delta * MM_PER_PULSE) / (dt * 1000.0f);  // m/s
}

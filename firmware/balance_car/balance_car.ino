#include "mpu6050.h"
#include "encoder.h"
#include "pid.h"
#include "wifi_config.h"
#include <WiFi.h>

// ── Motor pins ──────────────────────────────────────────────
#define MOTOR_L_IN1  14
#define MOTOR_L_IN2  15
#define MOTOR_L_ENA   2   // PWM
#define MOTOR_R_IN3  26
#define MOTOR_R_IN4  27
#define MOTOR_R_ENB  13   // PWM

#define PWM_CH_L  0
#define PWM_CH_R  1
#define PWM_FREQ  20000   // 20kHz, above audible range
#define PWM_BITS  8       // 0-255

// ── Control parameters ──────────────────────────────────────
#define LOOP_HZ        200
#define LOOP_DT        (1.0f / LOOP_HZ)
#define REPORT_EVERY   4            // 50Hz to host
#define TILT_CUTOFF    60.0f

// 0 = ANGLE_ONLY (tune PD first), 1 = FULL (angle + speed)
int ctrl_mode = 0;

// Mechanical zero offset
float PITCH_OFFSET = 3.8f;

// ── Position hold ────────────────────────────────────────────
// 位置积分（m），在速度环 setpoint 中作为误差来源
static float pos_m = 0.0f;
// 位置P增益：pos_m * POS_GAIN = 速度目标(m/s)
// 建议从 0.3 开始调，值越大位置越锁死但可能过激
float POS_GAIN = 0.3f;

// ── Differential balance ─────────────────────────────────────
// 左右轮速差补偿：diff_corr = DIFF_KP × (speed_l - speed_r)
// diff_corr 加到左轮、减到右轮，使两轮转速同步
// 建议从 5.0 开始调，值越大同步越紧但抖动越大
float DIFF_KP = 5.0f;

// ── PID instances ────────────────────────────────────────────
PID pid_angle(22.0f, 0.0f, 1.4f, -255, 255);
PID pid_speed(0.42f, 0.0f, 0.0f, -8, 8);

MPU6050 imu;

// ── WiFi / TCP ───────────────────────────────────────────────
static WiFiServer tcp_server(TCP_PORT);
static WiFiClient tcp_client;

// 广播一行数据到 Serial + TCP（如有连接）
static void bcast(const char *buf) {
    Serial.print(buf);
    if (tcp_client && tcp_client.connected()) {
        tcp_client.print(buf);
    }
}

// ── Timing ───────────────────────────────────────────────────
static hw_timer_t *timer = nullptr;
static volatile bool loop_flag = false;

static void IRAM_ATTR onTimer() { loop_flag = true; }

#define MOTOR_MIN_PWM  130  // L298N死区补偿，线性映射下限

// ── Motor helpers ─────────────────────────────────────────────
void motorSetLeft(int pwm) {
    if (pwm >= 0) {
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
    } else {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
        pwm = -pwm;
    }
    // 线性映射：PID输出[1,255] → 电机[MOTOR_MIN_PWM,255]，消除0→150跳变极限环
    if (pwm > 0) pwm = MOTOR_MIN_PWM + (int)(pwm * (255 - MOTOR_MIN_PWM) / 255);
    ledcWrite(PWM_CH_L, constrain(pwm, 0, 255));
}

void motorSetRight(int pwm) {
    if (pwm >= 0) {
        digitalWrite(MOTOR_R_IN3, HIGH);
        digitalWrite(MOTOR_R_IN4, LOW);
    } else {
        digitalWrite(MOTOR_R_IN3, LOW);
        digitalWrite(MOTOR_R_IN4, HIGH);
        pwm = -pwm;
    }
    if (pwm > 0) pwm = MOTOR_MIN_PWM + (int)(pwm * (255 - MOTOR_MIN_PWM) / 255);
    ledcWrite(PWM_CH_R, constrain(pwm, 0, 255));
}

void motorStop() {
    digitalWrite(MOTOR_L_IN1, LOW); digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN3, LOW); digitalWrite(MOTOR_R_IN4, LOW);
    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, 0);
}

// ── Command dispatcher ────────────────────────────────────────
// SET,kp,ki,kd        → update angle PID
// SETSPD,kp,ki,kd     → update speed PID
// OFFSET,value        → pitch zero offset
// MODE,0|1            → control mode
// CALIB               → IMU calibration
void dispatchLine(const String &line) {
    if (line.startsWith("SET,")) {
        float kp, ki, kd;
        if (sscanf(line.c_str(), "SET,%f,%f,%f", &kp, &ki, &kd) == 3) {
            pid_angle.setGains(kp, ki, kd);
            char buf[48]; snprintf(buf, sizeof(buf), "ACK,SET,%.3f,%.4f,%.3f\n", kp, ki, kd);
            bcast(buf);
        }
    } else if (line.startsWith("SETSPD,")) {
        float kp, ki, kd;
        if (sscanf(line.c_str(), "SETSPD,%f,%f,%f", &kp, &ki, &kd) == 3) {
            pid_speed.setGains(kp, ki, kd);
            char buf[48]; snprintf(buf, sizeof(buf), "ACK,SETSPD,%.3f,%.4f,%.3f\n", kp, ki, kd);
            bcast(buf);
        }
    } else if (line.startsWith("SETPOS,")) {
        float gain;
        if (sscanf(line.c_str(), "SETPOS,%f", &gain) == 1) {
            POS_GAIN = gain;
            char buf[32]; snprintf(buf, sizeof(buf), "ACK,SETPOS,%.4f\n", gain);
            bcast(buf);
        }
    } else if (line.startsWith("SETDIFF,")) {
        float gain;
        if (sscanf(line.c_str(), "SETDIFF,%f", &gain) == 1) {
            DIFF_KP = gain;
            char buf[32]; snprintf(buf, sizeof(buf), "ACK,SETDIFF,%.4f\n", gain);
            bcast(buf);
        }
    } else if (line == "ZEROPOS") {
        pos_m = 0.0f;
        bcast("ACK,ZEROPOS\n");
    } else if (line.startsWith("OFFSET,")) {
        float off;
        if (sscanf(line.c_str(), "OFFSET,%f", &off) == 1) {
            PITCH_OFFSET = off;
            pid_angle.reset(); pid_speed.reset();
            pos_m = 0.0f;
            char buf[32]; snprintf(buf, sizeof(buf), "ACK,OFFSET,%.3f\n", off);
            bcast(buf);
        }
    } else if (line.startsWith("MODE,")) {
        int m;
        if (sscanf(line.c_str(), "MODE,%d", &m) == 1) {
            ctrl_mode = constrain(m, 0, 1);
            pid_angle.reset(); pid_speed.reset();
            char buf[24]; snprintf(buf, sizeof(buf), "ACK,MODE,%d\n", ctrl_mode);
            bcast(buf);
        }
    } else if (line == "CALIB") {
        motorStop();
        bcast("INFO,Calibrating IMU...\n");
        imu.calibrate(500);
        bcast("INFO,Calibration done\n");
    }
}

// ── Serial command parser ─────────────────────────────────────
static String _serial_buf;

void parseSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            _serial_buf.trim();
            String line = _serial_buf; _serial_buf = "";
            if (line.length() > 0) dispatchLine(line);
        } else if (c != '\r') {
            _serial_buf += c;
            if (_serial_buf.length() > 64) _serial_buf = "";
        }
    }
}

// ── TCP command parser ────────────────────────────────────────
static String _tcp_buf;

void parseTCP() {
    // 接受新连接（同时只允许一个客户端）
    if (!tcp_client || !tcp_client.connected()) {
        WiFiClient nc = tcp_server.accept();
        if (nc) {
            tcp_client = nc;
            bcast("INFO,TCP client connected\n");
        }
    }
    if (!tcp_client || !tcp_client.connected()) return;

    while (tcp_client.available()) {
        char c = (char)tcp_client.read();
        if (c == '\n') {
            _tcp_buf.trim();
            String line = _tcp_buf; _tcp_buf = "";
            if (line.length() > 0) dispatchLine(line);
        } else if (c != '\r') {
            _tcp_buf += c;
            if (_tcp_buf.length() > 64) _tcp_buf = "";
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN3, OUTPUT); pinMode(MOTOR_R_IN4, OUTPUT);
    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_BITS);
    ledcAttachPin(MOTOR_L_ENA, PWM_CH_L);
    ledcAttachPin(MOTOR_R_ENB, PWM_CH_R);
    motorStop();

    // Encoders
    encoderInit();

    // IMU
    if (!imu.begin()) {
        Serial.println("ERROR,MPU6050 not found. Check SDA=GPIO21 SCL=GPIO22 and 3.3V power");
        delay(3000);
        ESP.restart();
    }
    Serial.println("INFO,MPU6050 OK");

    // WiFi
    Serial.printf("INFO,WiFi connecting to %s...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int wifi_tries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_tries < 20) {
        delay(500); wifi_tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("INFO,WiFi OK IP=%s\n", WiFi.localIP().toString().c_str());
        tcp_server.begin();
        Serial.printf("INFO,TCP server listening port %d\n", TCP_PORT);
    } else {
        Serial.println("INFO,WiFi failed, serial only");
    }

    // Warm up filter
    for (int i = 0; i < 200; i++) {
        imu.update(LOOP_DT);
        delay(5);
    }

    // 200Hz hardware timer
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000 / LOOP_HZ, true);
    timerAlarmEnable(timer);

    Serial.println("INFO,Balance car ready");
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {
    // TCP 接受新连接/读命令在主循环里轮询（不占控制帧时间）
    parseTCP();

    if (!loop_flag) return;
    loop_flag = false;

    parseSerial();

    // 1. Update IMU
    imu.update(LOOP_DT);
    float pitch = imu.getPitch() - PITCH_OFFSET;

    // 2. Safety cutoff
    if (fabsf(pitch) > TILT_CUTOFF) {
        motorStop();
        pid_angle.reset();
        pid_speed.reset();
        pos_m = 0.0f;   // 摔倒后归零位置，重新站稳后从当前位置开始
        static int cutoff_cnt = 0;
        if (++cutoff_cnt >= REPORT_EVERY) {
            cutoff_cnt = 0;
            char buf[140];
            snprintf(buf, sizeof(buf), "DATA,%.3f,0.0000,0.0000,0,0,%.3f,%.4f,%.4f,%d,%.3f,%.4f,%.4f,%.4f\n",
                pitch, pid_angle.kp, pid_angle.ki, pid_angle.kd, ctrl_mode,
                pid_speed.kp, pid_speed.ki, pid_speed.kd, pos_m);
            bcast(buf);
        }
        return;
    }

    // 3. Speed measurement
    // 左轮安装方向与右轮相反（面对面），取反使前进方向符号一致
    float speed_l = -encoderSpeedLeft(LOOP_DT);
    float speed_r = encoderSpeedRight(LOOP_DT);
    float speed_avg = (speed_l + speed_r) * 0.5f;

    // 4. Position integration + speed setpoint
    //    pos_m 在安全截止时不归零，MODE切换/OFFSET/ZEROPOS时归零
    float angle_correction = 0.0f;
    if (ctrl_mode == 1) {
        // 位置积分（m）
        pos_m += speed_avg * LOOP_DT;
        // 速度目标 = -pos_m * POS_GAIN（被推走→速度环要把车拉回来）
        pid_speed.setpoint = -pos_m * POS_GAIN;
        angle_correction = pid_speed.compute(speed_avg, LOOP_DT);
    }

    // 5. Angle PID → PWM
    pid_angle.setpoint = angle_correction;
    float pwm_out = pid_angle.compute(pitch, LOOP_DT);

    // 6. Drive motors
    // 差速补偿：左右轮速差 × DIFF_KP，加到慢轮、减到快轮
    float diff_corr = DIFF_KP * (speed_l - speed_r);
    diff_corr = constrain(diff_corr, -30.0f, 30.0f);  // 限幅，避免过补偿
    motorSetLeft((int)(pwm_out - diff_corr));
    motorSetRight((int)(pwm_out + diff_corr));

    // 7. Report to host at 50Hz
    // 格式: DATA,pitch,sl,sr,pwm_l,pwm_r,kp,ki,kd,mode,spd_kp,spd_ki,spd_kd,pos_m
    static int report_cnt = 0;
    if (++report_cnt >= REPORT_EVERY) {
        report_cnt = 0;
        char buf[140];
        snprintf(buf, sizeof(buf), "DATA,%.3f,%.4f,%.4f,%d,%d,%.3f,%.4f,%.4f,%d,%.3f,%.4f,%.4f,%.4f\n",
            pitch, speed_l, speed_r,
            (int)pwm_out, (int)pwm_out,
            pid_angle.kp, pid_angle.ki, pid_angle.kd,
            ctrl_mode,
            pid_speed.kp, pid_speed.ki, pid_speed.kd,
            pos_m);
        bcast(buf);
    }
}

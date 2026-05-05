# 平衡车 AI 智能调参系统

基于 ESP32 的两轮自平衡小车，配套 Python 上位机，集成 **DeepSeek AI 多智能体**自动调整 PID 参数。

![Python](https://img.shields.io/badge/Python-3.8+-blue)
![ESP32](https://img.shields.io/badge/ESP32-Arduino-red)
![License](https://img.shields.io/badge/License-MIT-green)

---

## 功能特性

- **实时数据可视化** — Pitch 角度、左右轮速、PWM 输出三路波形同步显示
- **串口 / WiFi TCP 双模通信** — 支持 USB 串口和无线连接，断线自动重连
- **多智能体 PID 自动调参**
  - **Planner**：分析当前振荡状态，决定调角度环还是速度环
  - **Tuner**：在安全范围内给出具体参数
  - **Reflector**：每次调参后提炼经验写入记忆库，越调越聪明
- **参数持久化** — 调好的参数断电不丢，重连自动下发到 ESP32
- **调参报告生成** — 一键生成 Markdown 报告，凝练历史经验

---

## 硬件清单

| 组件 | 型号 / 规格 |
|------|------------|
| 主控 | ESP32 DevKit（38pin） |
| IMU | MPU-6050（I²C，SDA=GPIO21，SCL=GPIO22） |
| 电机驱动 | L298N |
| 左电机 | IN1=GPIO14，IN2=GPIO15，ENA=GPIO2（PWM） |
| 右电机 | IN3=GPIO26，IN4=GPIO27，ENB=GPIO13（PWM） |
| 编码器 | 左轮 A=GPIO34，B=GPIO35；右轮 A=GPIO32，B=GPIO33 |
| 电源 | 7.4V 锂电池（建议 2S 18650） |

---

## 目录结构

```
├── firmware/
│   └── balance_car/
│       ├── balance_car.ino      # 主程序
│       ├── mpu6050.h / .cpp     # IMU 驱动（互补滤波）
│       ├── encoder.h / .cpp     # 编码器驱动
│       ├── pid.h / .cpp         # PID 控制器
│       └── wifi_config.example.h  # WiFi 配置模板（复制后改名）
├── host/
│   ├── main.py                  # 上位机主程序（Tkinter GUI）
│   ├── agent.py                 # AI 多智能体调参逻辑
│   ├── requirements.txt         # Python 依赖
│   └── config.example.json      # 配置文件模板（复制后改名）
└── README.md
```

---

## 快速开始

### 1. 固件烧录

```bash
# 复制 WiFi 配置模板
cp firmware/balance_car/wifi_config.example.h firmware/balance_car/wifi_config.h
# 编辑 wifi_config.h，填入你的 WiFi SSID 和密码

# 用 Arduino IDE 打开 firmware/balance_car/balance_car.ino
# 选择开发板: ESP32 Dev Module，烧录
```

### 2. 上位机安装

```bash
cd host
pip install -r requirements.txt

# 复制配置模板
cp config.example.json config.json
# 编辑 config.json，填入 DeepSeek API Key
```

> 获取 DeepSeek API Key：https://platform.deepseek.com/

### 3. 运行

```bash
cd host
python main.py
```

---

## 使用说明

### 连接小车

- **串口模式**：选择对应 COM 口，点击「连接」
- **WiFi 模式**：填入 ESP32 的 IP 地址（串口监视器可查），选择 WiFi 模式后连接

### 调参流程

1. **设置零点偏移** — 将小车竖立静止，点击「用当前值」自动填入 Pitch 偏移
2. **IMU 校准** — 小车放平静止，点击「IMU 校准」
3. **模式1：只调角度环** — 先让小车能站稳
4. **智能体调参** — 点击「▶ 智能体调参」，AI 自动分析并下发参数，5 秒后验证效果
5. **模式2：完整双环** — 角度环稳定后切换，继续调速度环

### 串口协议

上位机通过以下指令控制 ESP32：

| 指令 | 说明 |
|------|------|
| `SET,kp,ki,kd` | 设置角度环 PID |
| `SETSPD,kp,ki,kd` | 设置速度环 PID |
| `OFFSET,value` | 设置 Pitch 零点偏移 |
| `MODE,0\|1` | 切换控制模式（0=角度环，1=双环） |
| `CALIB` | IMU 校准 |
| `SETPOS,gain` | 设置位置保持增益 |
| `SETDIFF,gain` | 设置差速补偿增益 |
| `ZEROPOS` | 位置归零 |

ESP32 上报格式（50Hz）：
```
DATA,pitch,speed_l,speed_r,pwm_l,pwm_r,kp,ki,kd,mode,spd_kp,spd_ki,spd_kd,pos_m
```

---

## AI 调参原理

```
传感器数据
    ↓
Planner（决策）→ 分析振荡状态，选择调角度环/速度环/观察
    ↓
Tuner（参数）→ 在安全范围内给出具体 Kp/Ki/Kd
    ↓
下发到 ESP32，等待 5 秒
    ↓
验证效果（std 对比）→ 变差则自动回滚
    ↓
Reflector（反思）→ 提炼经验写入 tuning_memory.json
    ↓
下次 Planner 优先参考历史经验
```

---

## 调参经验（实测）

根据本项目实际调参记录（50 次迭代），推荐起始参数：

| 环路 | Kp | Ki | Kd |
|------|----|----|----|
| 角度环 | 23.0 | 0 | 1.39 |
| 速度环 | 0.42 | 0 | 0 |

- 速度环 Kp 不要超过 0.43，Ki 保持 0
- 角度环发散时先降 Kp，再增 Kd
- 稳态误差明显时才引入微弱 Ki（≤ 0.001）

---

## 依赖

**上位机（Python）**
- `pyserial` — 串口通信
- `matplotlib` — 实时波形
- `requests` — DeepSeek API 调用

**固件（Arduino）**
- ESP32 Arduino Core
- 内置 WiFi 库

---

## License

MIT

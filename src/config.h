// ===================== 全局配置参数 (config.h) =====================
// 所有引脚映射、控制常量、运动学参数集中管理
#pragma once

#include <Arduino.h>

// ------------------- [1. 硬件引脚精确映射] -------------------
// 接收机通道
#define PX4_CH1_PIN  14   // 右手左右摇杆 (对应底盘转向)
#define PX4_CH2_PIN  13   // 右手前后摇杆 (对应底盘前进/后退)
#define PX4_CH9_PIN  12   // 模式切换开关 (低电平飞行，高电平陆地)
#define PX4_CH10_PIN 11   // 清洁电机物理开关 & 自主清扫触发器

// 底盘电机驱动引脚
#define STBY_PIN      4
#define PWMA_PIN      5    // 左前电机 PWM
#define AIN1_PIN      7    // 左前电机 IN1
#define AIN2_PIN      6    // 左前电机 IN2
#define PWMB_PIN      48   // 左后电机 PWM
#define BIN1_PIN      21   // 左后电机 IN1
#define BIN2_PIN      47   // 左后电机 IN2
#define PWMC_PIN      17   // 右后电机 PWM
#define CIN1_PIN      8    // 右后电机 IN1
#define CIN2_PIN      18   // 右后电机 IN2
#define PWMD_PIN      1    // 右前电机 PWM
#define DIN1_PIN      42   // 右前电机 IN1（原 GPIO 37 连到内部 Flash，已迁移）
#define DIN2_PIN      2    // 右前电机 IN2

// 清洁电机驱动引脚 (第二块 TB6612，1 路，留 1 路备用)
#define CLEAN_STBY_PIN    10   // TB6612 STBY (独立使能)
#define CLEAN_PWM_PIN     9    // TB6612 PWMC
#define CLEAN_PWM_CH      4    // LEDC PWM 通道号 (主电机占用 0~3)
#define CLEAN_PWM_SPEED   128  // 清洁电机 PWM 占空比 (0~255)

// 霍尔编码器引脚
// GPIO 33~37 在 ESP32-S3-N16R8 模组内部连接 Octal PSRAM，不可外接！
// 右编码器改用 GPIO 40/41（J3 排针 11/12 脚）
#define ENC_L_A      15
#define ENC_L_B      16
#define ENC_R_A      41
#define ENC_R_B      40

// I2C MPU6050引脚
#define MPU_SDA_PIN   3
#define MPU_SCL_PIN   39
#define MPU6050_ADDR  0x68

// ------------------- [2. 常量与控制参数] -------------------
#define ERROR_LIMIT    800   // 位置环误差保护限幅 (单位：脉冲)
#define I_V_LIMIT      100   // 速度环积分限幅
#define CONTROL_PERIOD 20    // 控制周期 20ms (50Hz)
#define DEBUG_PRINT_INTERVAL 500 // 串口输出间隔 500ms

// 重力前馈与斜坡参数
#define PITCH_MAX_DEG      60.0f   // 前馈计算的最大俯仰角 (度)
#define PITCH_PWM_GAIN     (500.0f / PITCH_MAX_DEG)  // PWM/度 转换增益
#define FF_FADE_STEP       0.02f   // 重力前馈淡入步进 (每周期)
#define FF_FADE_CYCLES     50      // 淡入完成所需周期数 (= 1.0 / FF_FADE_STEP)

// 手动控制死区
#define THROTTLE_DEADBAND  80   // 油门通道死区 (±µs 相对于 1500)
#define STEERING_DEADBAND  50   // 转向通道死区 (±µs 相对于 1500)

// PID 参数
#define KP_V_FLAT    8.5f   // 速度环比例增益 (平地)
#define KP_V_SLOPE   4.0f   // 速度环比例增益坡道增量 (陡坡时附加)
#define MIN_RPM_FACTOR 0.46666667f  // 陡坡保留最小速比

// 前馈映射范围
#define FF_MAP_MIN   -45     // 前馈 PWM 最小值 (极限前倾)
#define FF_MAP_MAX    45     // 前馈 PWM 最大值 (极限后仰)

// 手动控制速度映射
#define THROTTLE_STEP_MIN  -50  // 油门最小步进
#define THROTTLE_STEP_MAX   50  // 油门最大步进
#define STEERING_STEP_MIN  -25  // 转向最小步进
#define STEERING_STEP_MAX   25  // 转向最大步进

// 位置误差告警阈值 (占 ERROR_LIMIT 比例)
#define POS_OVERRUN_THRESHOLD 0.85f

// ------------------- [3. 安全与可靠性参数] -------------------
#define SAFETY_TILT_MAX    45.0f   // 最大允许倾角 (度)，超限→急停
#define SAFETY_SIGNAL_TIMEOUT 500  // 接收机信号超时判定 (ms)
#define SAFETY_STALL_MS    1000     // 堵转判定持续时长 (ms)
#define SAFETY_STALL_PWM   300     // 视为堵转的持续饱和输出下限 (PWM 绝对值)
#define SAFETY_PHASE_TIMEOUT 10000 // 自动清扫单阶段最大耗时 (ms)
#define WDT_TIMEOUT        3000    // 看门狗超时 (ms)，超限→硬件复位

// 编码器与速度换算常量
#define ENCODER_PPR   13
#define GEAR_RATIO    34
#define ENCODER_MULT  4      // 计数倍频 (4倍频)
#define MAX_RPM       250.0f // 电机额定转速 (RPM)

// ------------------- [4. 串口通信（ESP32 ↔ Orin Nano UART1）] -------------------
// 物理引脚
#define COMM_RX_PIN      44     // UART1 RX (接 Orin Nano TX, Pin 8)
#define COMM_TX_PIN      43     // UART1 TX (接 Orin Nano RX, Pin 10)
#define COMM_BAUD        921600 // 波特率 (7+15 字节/帧, ~190µs 传输时间)

// 通信帧参数
#define COMM_FRAME_PERIOD    20   // 收发周期 20ms (50Hz, 与控制周期同步)
#define COMM_SYNC_DOWN    0xAA   // 下行帧头 (ROS2 → ESP32)
#define COMM_SYNC_UP      0xBB   // 上行帧头 (ESP32 → ROS2)
#define COMM_DOWN_LEN        7   // 下行帧字节数
#define COMM_UP_LEN         15   // 上行帧字节数

// 底盘控制模式 (与下行帧 MODE 字段对应)
#define MODE_RELEASE        0   // 释放：电机无动力，自由滑行
#define MODE_MANUAL         1   // 手动遥控：RC 接收机控制 (默认)
#define MODE_ROS2_AUTO      2   // ROS2 自主导航：速度指令来自 Orin Nano
#define MODE_ESTOP          3   // 急停：ESP32 本地急停 (不受上位机控制)

// 通信超时保护
#define COMM_RX_TIMEOUT   200   // 下行帧超时 (ms), 超时自动切回手动模式

// ------------------- [5. 自主"弓"字形清扫核心参数] -------------------
#define AUTO_LINE_PULSES   8000  // 单行直行距离 (脉冲数)
#define AUTO_TURN_PULSES   1150  // 原地自旋90度所需单侧脉冲
#define AUTO_ROW_PULSES    1500  // 换行横向平移距离
#define AUTO_SPEED         15    // 自动直行速度步进
#define AUTO_TURN_SPEED    12    // 自动自旋速度步进
#define MAX_ROWS           6     // 总清扫行数
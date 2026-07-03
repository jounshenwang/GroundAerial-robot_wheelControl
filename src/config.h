// ===================== 全局配置参数 (config.h) =====================
// 所有引脚映射、控制常量、运动学参数集中管理
#pragma once

#include <Arduino.h>

// ------------------- [1. 硬件引脚精确映射] -------------------
// 接收机通道
#define PX4_CH1_PIN  13   // 右手左右摇杆 (对应底盘转向)
#define PX4_CH2_PIN  14   // 右手前后摇杆 (对应底盘前进/后退) — 避免使用 GPIO12 (MTDI strapping)
#define PX4_CH9_PIN  11   // 模式切换开关 (低电平飞行，高电平陆地)
#define PX4_CH10_PIN 10   // 清洁电机物理开关 & 自主清扫触发器

// 底盘电机驱动引脚
#define STBY_PIN      4
#define PWMA_PIN      5    // 左前电机 PWM
#define AIN1_PIN      7    // 左前电机 IN1
#define AIN2_PIN      6    // 左前电机 IN2
#define PWMB_PIN      17   // 左后电机 PWM
#define BIN1_PIN      18   // 左后电机 IN1
#define BIN2_PIN      21   // 左后电机 IN2
#define PWMC_PIN      1    // 右后电机 PWM
#define CIN1_PIN      2    // 右后电机 IN1
#define CIN2_PIN      42   // 右后电机 IN2
#define PWMD_PIN      38   // 右前电机 PWM
#define DIN1_PIN      37   // 右前电机 IN1
#define DIN2_PIN      39   // 右前电机 IN2

// 清洁电机控制引脚
#define CLEAN_MOTOR_PIN 47

// 霍尔编码器引脚
#define ENC_L_A      15
#define ENC_L_B      16
#define ENC_R_A      36
#define ENC_R_B      35

// I2C MPU6050引脚
#define MPU_SDA_PIN   8
#define MPU_SCL_PIN   3
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
#define SAFETY_STALL_MS    500     // 堵转判定持续时长 (ms)
#define SAFETY_STALL_PWM   250     // 视为堵转的持续饱和输出下限 (PWM 绝对值)
#define SAFETY_PHASE_TIMEOUT 10000 // 自动清扫单阶段最大耗时 (ms)
#define WDT_TIMEOUT        3000    // 看门狗超时 (ms)，超限→硬件复位

// 编码器与速度换算常量
#define ENCODER_PPR   13
#define GEAR_RATIO    34
#define ENCODER_MULT  4      // 计数倍频 (4倍频)
#define MAX_RPM       250.0f // 电机额定转速 (RPM)

// ------------------- [4. 自主"弓"字形清扫核心参数] -------------------
#define AUTO_LINE_PULSES   8000  // 单行直行距离 (脉冲数)
#define AUTO_TURN_PULSES   1150  // 原地自旋90度所需单侧脉冲
#define AUTO_ROW_PULSES    1500  // 换行横向平移距离
#define AUTO_SPEED         15    // 自动直行速度步进
#define AUTO_TURN_SPEED    12    // 自动自旋速度步进
#define MAX_ROWS           6     // 总清扫行数
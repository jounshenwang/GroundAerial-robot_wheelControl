// ===================== 串口通信模块 (comm.h) =====================
// ESP32 (UART1) ↔ Jetson Orin Nano (serial_bridge) 二进制帧协议
//
// 下行帧 (ROS2 → ESP32): SYNC(1) + L_VEL(2) + R_VEL(2) + MODE(1) + CS(1) = 7B
// 上行帧 (ESP32 → ROS2): SYNC(1) + ENC_L(4) + ENC_R(4) + PITCH(2) + FAULT(2) + STATE(1) + CS(1) = 15B
//
// 详见架构文档: docs/ESP32与ROS2架构分工.md §4
#pragma once

#include <Arduino.h>
#include "config.h"

namespace Comm {

// ======================== 帧结构定义 ========================

/// 下行帧 (ROS2 → ESP32) — 7 字节
struct DownFrame {
    uint8_t  sync;      // 帧头 0xAA
    int16_t  vTargetL;  // 左轮目标速度 (编码器脉冲/控制周期, int16)
    int16_t  vTargetR;  // 右轮目标速度
    uint8_t  mode;      // 控制模式: MODE_RELEASE / MANUAL / ROS2_AUTO / ESTOP
    uint8_t  checksum;  // 前 6 字节 XOR 校验
} __attribute__((packed));

/// 上行帧 (ESP32 → ROS2) — 15 字节
struct UpFrame {
    uint8_t  sync;      // 帧头 0xBB
    int32_t  encL;      // 左编码器累计脉冲数
    int32_t  encR;      // 右编码器累计脉冲数
    int16_t  pitch;     // 俯仰角 ×100 (°), 正=前倾
    uint16_t fault;     // 故障位掩码 (与 Safety::FaultFlag 对齐)
    uint8_t  state;     // 安全状态 (SafetyState 枚举值)
    uint8_t  checksum;  // 前 14 字节 XOR 校验
} __attribute__((packed));

// ======================== 公共接口 ========================

/**
 * @brief 初始化 UART1 串口 (921600, 8N1)
 * 在 setup() 中调用，位于 Serial.begin() 之后
 */
void commInit();

/**
 * @brief 每控制周期调用 (50Hz)
 * 接收下行帧 + 发送上行帧
 * 应在控制循环的开头调用
 */
void commUpdate();

/// 获取解析后的左轮目标速度 (编码器脉冲/控制周期)
int16_t getTargetVelocityL();

/// 获取解析后的右轮目标速度 (编码器脉冲/控制周期)
int16_t getTargetVelocityR();

/// 获取当前控制模式
uint8_t getMode();

/// 获取距上次有效接收的毫秒数 (用于超时判断)
uint32_t getRxAge();

} // namespace Comm

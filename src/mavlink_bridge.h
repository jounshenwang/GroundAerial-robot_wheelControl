// ===================== MAVLink 飞控通信模块 (mavlink_bridge.h) =====================
// 通过 UART2 (GPIO38/39) 与 Pixhawk TELEM2 交换 MAVLink v2 协议消息
//
// 核心功能:
//   ① HEARTBEAT 双向 — 维持 Mavlink 链路，监控飞控在线和武装状态
//   ② COMMAND_LONG (400)  — MAV_CMD_COMPONENT_ARM_DISARM → 锁死/解锁飞控电机
//   ③ COMMAND_LONG (185)  — MAV_CMD_DO_FLIGHTTERMINATION → 紧急终止飞行
//   ④ COMMAND_ACK 接收 — 确认飞控是否已执行命令
//
// 物理: UART2 (Serial2), 3.3V TTL 直连 Pixhawk TELEM2
//    ESP32 TX(GPIO38) → Pixhawk TELEM2 Pin 2 (RX)
//    ESP32 RX(GPIO39) → Pixhawk TELEM2 Pin 3 (TX)
//    ESP32 GND         → Pixhawk TELEM2 Pin 6 (GND)
//
// 协议: MAVLink v2 (起始字节 0xFD), 仅实现本系统需要的 3 条消息
//       无外部库依赖，帧编解码全部自实现
//
// 禁用: 注释 config.h 中的 MAVLINK_ENABLED → 所有函数退化为空桩
#pragma once

#include <Arduino.h>
#include "config.h"

namespace MavlinkBridge {

/// 初始化 UART2 (Serial2) 与接收状态机
void begin();

/// 每控制周期调用 (50Hz): 接收帧 + 定时发送 HEARTBEAT
void update();

/// @brief 向飞控发送 arm/disarm 命令
/// @param arm  true=解锁, false=上锁 (锁死所有无刷电机)
/// @return 飞控在线时返回 true
bool sendArmDisarm(bool arm);

/// @brief 紧急终止飞行 (飞控锁死所有电机，部分固件需重启恢复)
/// @return 飞控在线时返回 true
bool sendFlightTermination();

/// 飞控链路是否在线 (MAVLINK_LINK_TIMEOUT 内收到过飞控 HEARTBEAT)
bool isFcuConnected();

/// 飞控当前是否武装 (从 HEARTBEAT base_mode 解析)
bool isFcuArmed();

/// 自动发现的飞控系统 ID
uint8_t fcuSysId();

} // namespace MavlinkBridge

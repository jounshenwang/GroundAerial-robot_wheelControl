// ===================== 安全与可靠性模块 (safety.h) =====================
// 自检 / 急停 / 故障降级 / 运行时监控
#pragma once

#include <Arduino.h>
#include "config.h"

// ---------- 故障标志位 (位掩码) ----------
// 运行时监控自动检测:   RX_SIGNAL / OVER_TILT / STALL_L/R
// 主循环中手动置位:     POS_OVERRUN
// 预留 (尚无注入逻辑):  IMU / ENCODER_L/R / BAT_LOW / MOTOR_DRV
enum FaultFlag : uint16_t {
    FAULT_NONE        = 0x0000,
    FAULT_IMU         = 1 << 0,   // [预留] IMU 数据冻结
    FAULT_ENCODER_L   = 1 << 1,   // [预留] 左编码器异常
    FAULT_ENCODER_R   = 1 << 2,   // [预留] 右编码器异常
    FAULT_RX_SIGNAL   = 1 << 3,   // 接收机信号丢失 (所有通道冻结)
    FAULT_MOTOR_DRV   = 1 << 4,   // [预留] 电机驱动故障
    FAULT_BAT_LOW     = 1 << 5,   // [预留] 电池电压过低
    FAULT_OVER_TILT   = 1 << 6,   // 倾角超限
    FAULT_STALL_L     = 1 << 7,   // 左轮堵转
    FAULT_STALL_R     = 1 << 8,   // 右轮堵转
    FAULT_POS_OVERRUN = 1 << 9,   // 位置误差超限 (告警类, 不直接触发急停)
};

// ---------- 安全状态机 ----------
enum SafetyState : uint8_t {
    SAFETY_INIT     = 0,  // 上电自检中
    SAFETY_NORMAL   = 1,  // 正常运行
    SAFETY_WARNING  = 2,  // 有告警但可运行 (如电池偏低)
    SAFETY_DEGRADED = 3,  // 降级模式 (部分功能受限)
    SAFETY_ESTOP    = 4,  // 急停：所有电机动力切断 + 主动刹车
};

namespace Safety {

extern SafetyState state;       ///< 当前安全状态
extern uint16_t    faultFlags;  ///< 活跃故障位掩码

/**
 * @brief 上电自检 (Power-On Self Test)
 * 检查接收机信号有效性、初始化看门狗
 * 应在所有 begin() 调用之后执行
 */
void runPOST();

/**
 * @brief 每控制周期调用，运行时持续监控
 * @param speedL  左轮速度 (脉冲/控制周期)
 * @param speedR  右轮速度
 * @param outL    左轮 PID 输出 (-255~255)
 * @param outR    右轮 PID 输出
 *
 * 检测项：信号丢失 / 倾角超限 / 堵转 / 状态降级
 */
void runtimeMonitor(int speedL, int speedR, int outL, int outR);

/// 手动触发急停 (切断所有电机，主动刹车)
void triggerEStop(const char* reason);

/**
 * @brief 从急停恢复 (需遥控器执行复位手势)
 *   ch9 < 1200 (飞行) → ch9 > 1700 (地面) 切换一次
 * @return true 恢复成功回到 SAFETY_NORMAL
 */
bool tryRecover();

/// 每控制周期无条件喂看门狗 (必须放在 ESTOP 检查之前)
void feedWatchdog();

/// 输出安全状态和故障信息到 Serial
void printStatus();

/// 检查是否存在致命故障
bool hasCriticalFault();

} // namespace Safety

// ===================== 自主"弓"字形清扫状态机 (autoclean.h) =====================
#pragma once

#include <Arduino.h>
#include "config.h"

/// 清扫状态枚举
enum AutoCleanState : uint8_t {
    CLEAN_IDLE     = 0,  // 待命 / 手动遥控
    LINE_FORWARD   = 1,  // 正向直行 (上坡)
    TURN_RIGHT_1   = 2,  // 第1次右转90°
    SHIFT_RIGHT    = 3,  // 向右横移换行
    TURN_RIGHT_2   = 4,  // 第2次右转90° (完成180°调头)
    LINE_BACKWARD  = 5,  // 反向直行 (下坡)
    TURN_LEFT_1    = 6,  // 第1次左转90°
    SHIFT_LEFT     = 7,  // 向左横移换行
    TURN_LEFT_2    = 8,  // 第2次左转90° (完成反向180°)
    CLEAN_DONE     = 9   // 全部完成，安全驻车
};

/// 状态机单周期输出
struct AutoCleanResult {
    int  moveStep;   // 移动步进 (正=前进)
    int  steerStep;  // 转向步进 (正=右转)
    bool active;     // true = 状态机正在运行
};

// ── 外部可访问的状态变量 ──
extern AutoCleanState autoState;
extern int rowCount;

/// 切换状态并锁定当前编码器位置为阶段起点
void changeAutoState(AutoCleanState newState, long countL, long countR);

/// 每个控制周期调用一次 (仅在自动模式下)
AutoCleanResult autoCleanUpdate(long countL, long countR);

/// 重置为待命状态 (手动接管 / 模式切换时调用)
void autoCleanReset();
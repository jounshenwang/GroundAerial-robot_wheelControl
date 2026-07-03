// ===================== 编码器模块 (encoder.h) =====================
// 霍尔编码器中断服务与计数读取
#pragma once

#include <Arduino.h>
#include "config.h"

namespace Encoder {

/// 左右轮编码器脉冲计数（由 ISR 更新，volatile 保证跨核心可见）
extern volatile long count_l;
extern volatile long count_r;

/// 原子读取左编码器计数值 (关中断防止 ISR 撕裂)
static inline long readL() {
    noInterrupts();
    long val = count_l;
    interrupts();
    return val;
}

/// 原子读取右编码器计数值
static inline long readR() {
    noInterrupts();
    long val = count_r;
    interrupts();
    return val;
}

/// 原子读取左右编码器计数值 (同一临界区内, 保证左右一致性)
static inline void readPair(long &l, long &r) {
    noInterrupts();
    l = count_l;
    r = count_r;
    interrupts();
}

/// 初始化：绑定编码器引脚中断（四边沿触发，4倍频）
void begin();

} // namespace Encoder
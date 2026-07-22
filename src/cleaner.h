// ===================== 清洁电机控制模块 (cleaner.h) =====================
// 封装清洁电机（额外电机/继电器控制）的开关接口
#pragma once

#include <Arduino.h>
#include "config.h"

namespace Cleaner {

/// 初始化清洁电机引脚 (应在 setup 中调用)
void begin();

/// 设置清洁电机开关状态
void set(bool on);

/// 快捷关闭
inline void off() { set(false); }

/// 快捷开启
inline void on()  { set(true); }

} // namespace Cleaner

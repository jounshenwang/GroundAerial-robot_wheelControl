// ===================== 遥控接收机模块 (receiver.h) =====================
// PX4 PWM 接收机通道解码，由中断测量高电平脉宽
#pragma once

#include <Arduino.h>
#include "config.h"

namespace Receiver {

/// 4 个遥控通道值，单位 µs，范围 1000–2000，中立 1500
extern volatile uint32_t ch1;   // 转向
extern volatile uint32_t ch2;   // 前进/后退
extern volatile uint32_t ch9;   // 模式切换 (≤1700 飞行 / >1700 陆地)
extern volatile uint32_t ch10;  // 清洁开关 & 自主清扫触发

/// 绑定引脚中断
void begin();

} // namespace Receiver
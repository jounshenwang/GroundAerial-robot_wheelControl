// ===================== 电机驱动模块 (motor.h) =====================
// TB6612 四路独立电机驱动封装
#pragma once

#include <Arduino.h>
#include "config.h"

/// 初始化电机控制引脚与 LEDC PWM
void motorBegin();

/**
 * @brief 设置单路电机输出
 * @param out     PWM 值 -255~255 (正=前进，负=后退)
 * @param p1      IN1 引脚编号
 * @param p2      IN2 引脚编号
 * @param channel LEDC 通道号 (0~3)
 */
void motorSet(int out, int p1, int p2, int channel);

/// 左右差分输出到四路电机 (同侧信号一致)
void motorSetDifferential(int left, int right);

/// 停止所有电机
/// @param activeBrake true=防溜坡短路制动, false=完全释放
void motorStopAll(bool activeBrake);
// ===================== MPU6050 姿态传感器 (imu.h) =====================
// 加速度/陀螺仪读取 + 互补滤波俯仰解算
#pragma once

#include <Arduino.h>

namespace IMU {

/// 当前俯仰角度 (度)，正=前倾
extern float angle_pitch;
extern bool dataValid;          ///< IMU 数据是否有效（I2C 故障/恢复期间 false）

/// 初始化 MPU6050 (Wire 总线 + 唤醒)
/// @return true=成功
bool begin();

/// 每控制周期调用：读取原始数据 → 互补滤波 → 更新 angle_pitch
void update();

} // namespace IMU

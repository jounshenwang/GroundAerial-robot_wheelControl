// ===================== 串级 PID 控制器 (pid.h) =====================
#pragma once

#include <Arduino.h>
#include "config.h"

/// 串级 PID 结构体（位置环外环 + 速度环内环）
/// 集成抗积分饱和 (Anti-Windup) 机制：条件积分法
struct CascadePID {
    // ---------- 位置环参数 (P + I) ----------
    float kp_p       = 1.2;
    float ki_p       = 0.01;
    float p_integral = 0;
    float p_imax     = 50;      ///< 位置环积分上限 (对称)

    // ---------- 速度环参数 (PID) ----------
    float kp_v       = 8.5;
    float ki_v       = 1.2;
    float kd_v       = 0.5;
    float v_integral = 0;
    float v_imax     = 100;     ///< 速度环积分上限 (对称, = I_V_LIMIT)
    int v_last_speed = 0;       ///< 上一周期实际速度 (微分先行, int 防截断)

    // ---------- 抗积分饱和状态 (调试用) ----------
    bool  p_saturated = false;  ///< 上一周期位置环输出是否饱和
    bool  v_saturated = false;  ///< 上一周期速度环输出是否饱和

    /// 清空所有积分与误差历史
    void reset();

    /**
     * @brief 执行串级 PID 计算（含抗积分饱和）
     *
     * 抗积分饱和策略 —— 条件积分 (Conditional Integration)：
     *   - 当本环输出饱和时，若积分与误差同号（即积分在加剧饱和方向），
     *     则暂停积分累积；若异号（积分正在回拉），则正常累积。
     *   - 这样做既防止了饱和方向的 windup，又保留了反向的快速退饱和能力。
     *
     * @param target      位置目标 (编码器脉冲)
     * @param actualPos   当前实际位置
     * @param actualSpeed 当前实际速度 (脉冲/控制周期)
     * @param maxV        速度目标最大限幅 (来自坡度自适应)
     * @param kpV         速度环比例增益 (来自坡度自适应)
     * @param feedForward 重力前馈补偿值 (由主控根据姿态计算)
     * @return int        电机 PWM 输出 (-255 ~ 255)
     */
    int compute(long target, long actualPos, int actualSpeed,
                float maxV, float kpV, float feedForward);
};

// 全局左右 PID 实例
extern CascadePID pidL;
extern CascadePID pidR;

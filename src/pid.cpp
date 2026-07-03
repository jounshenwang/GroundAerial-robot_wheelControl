// ===================== 串级 PID 控制器 (pid.cpp) =====================
// 集成条件积分抗饱和 (Conditional Integration Anti-Windup)
#include "pid.h"

CascadePID pidL;
CascadePID pidR;

void CascadePID::reset() {
    p_integral   = 0;
    v_integral   = 0;
    v_last_speed = 0;
    p_saturated  = false;
    v_saturated  = false;
}

int CascadePID::compute(long target, long actualPos, int actualSpeed,
                         float maxV, float kpV, float feedForward) {
    // ═══════════════════════════════════════════════════════════════
    //  外环 —— 位置环 (P + I)
    //  输入:  位置误差 →  输出: 速度目标 v_target (受 maxV 限幅)
    // ═══════════════════════════════════════════════════════════════
    float p_err = (float)(target - actualPos);
    float p_out_unlimited = kp_p * p_err + ki_p * p_integral;

    // 判断是否饱和 (速度指令触及 maxV 天花板)
    p_saturated = (p_out_unlimited > maxV || p_out_unlimited < -maxV);
    float v_target = constrain(p_out_unlimited, -maxV, maxV);

    // ── 条件积分抗饱和 ──
    // 只在两种情况下累积位置积分:
    //   ① 输出未饱和 —— 正常工作，积分辅助消除静差
    //   ② 积分与误差异号 —— 误差已反向，积分正在"退饱和"，不应阻止
    // 当输出饱和且误差方向与积分同号时，暂停积分 (防止 windup)
    if (!p_saturated || (p_err * p_integral < 0.0f)) {
        p_integral += p_err;
    }
    p_integral = constrain(p_integral, -p_imax, p_imax);   // 安全硬限幅

    // ═══════════════════════════════════════════════════════════════
    //  内环 —— 速度环 (PID) — 微分先行 (Derivative on Measurement)
    //  输入:  速度误差 →  输出: PWM 占空比 (-255 ~ 255)
    //
    //  微分先行：D 项仅作用于实际速度的变化，而非误差变化。
    //  当位置目标 (v_target) 突跳时，误差微分项会产生巨大的
    //  "微分冲击"（derivative kick），导致电机瞬态抖动。
    //  改为对实际速度求微分后，目标突跳仅通过 P/I 通道响应，
    //  响应平滑且无冲击。
    // ═══════════════════════════════════════════════════════════════
    float v_err = v_target - (float)actualSpeed;

    // PID 分量 (未加前馈)
    //  D 项 = kd_v * (v_last_speed - actualSpeed) ≡ -kd_v * d(actual)/dt
    float pid_out = kpV * v_err
                  + ki_v * v_integral
                  + kd_v * (v_last_speed - actualSpeed);
    v_last_speed = actualSpeed;

    // 最终输出 = PID 分量 + 重力前馈
    float out_unsat = pid_out + feedForward;
    v_saturated = (out_unsat > 255.0f || out_unsat < -255.0f);
    int out = (int)constrain(out_unsat, -255.0f, 255.0f);

    // ── 条件积分抗饱和 ──
    // 当 PWM 已达硬件上限 (±255) 时，继续积分不会带来更大的控制量，
    // 反而会在误差反转后造成超调。暂停同向积分，允许反向退饱和。
    if (!v_saturated || (v_err * v_integral < 0.0f)) {
        v_integral += v_err;
    }
    v_integral = constrain(v_integral, -v_imax, v_imax);

    return out;
}

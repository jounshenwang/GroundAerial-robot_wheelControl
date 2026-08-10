// ===================== 编码器模块 (encoder.cpp) =====================
#include "encoder.h"

volatile long Encoder::count_l = 0;
volatile long Encoder::count_r = 0;

// ---------- 中断服务函数 (IRAM_ATTR 确保在 IRAM 中执行) ----------
// 标准 x4 正交解码: A/B 任一沿变化都计数, 用 (前一状态, 当前状态) 查表判方向。
//   cur = (B << 1) | A, 索引 = (prev << 2) | cur, 表值 = +1 / -1 / 0(非法跳变)
// ⚠️ 修正: 旧实现 "A!=B ? +1 : -1" 在每个完整电气周期内 +1-1+1-1 互相抵消,
//   计数值永远在 0±1 附近打转, 正反转都无法累积 —— 这正是读数恒为 0/1 的根因。
static const int8_t QUAD_TABLE[16] = {
     0, -1, 1, 0,   // 00→00, 00→01, 00→10, 00→11
     1,  0, 0, -1,  // 01→00, 01→01, 01→10, 01→11
    -1,  0, 0, 1,   // 10→00, 10→01, 10→10, 10→11
     0,  1, -1, 0   // 11→00, 11→01, 11→10, 11→11
};
volatile uint8_t encL_prev = 0;  // 上一个 (B,A) 状态, 仅 ISR 内部使用
volatile uint8_t encR_prev = 0;

void IRAM_ATTR encL_ISR() {
    uint8_t cur = (uint8_t)((digitalRead(ENC_L_B) << 1) | digitalRead(ENC_L_A));
    Encoder::count_l += QUAD_TABLE[(encL_prev << 2) | cur];
    encL_prev = cur;
}
void IRAM_ATTR encR_ISR() {
    uint8_t cur = (uint8_t)((digitalRead(ENC_R_B) << 1) | digitalRead(ENC_R_A));
    Encoder::count_r += QUAD_TABLE[(encR_prev << 2) | cur];
    encR_prev = cur;
}

void Encoder::begin() {
    // 初始化前一状态, 避免首次中断用默认 0 误判方向
    encL_prev = (uint8_t)((digitalRead(ENC_L_B) << 1) | digitalRead(ENC_L_A));
    encR_prev = (uint8_t)((digitalRead(ENC_R_B) << 1) | digitalRead(ENC_R_A));
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), encL_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), encR_ISR, CHANGE);
}
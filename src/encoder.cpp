// ===================== 编码器模块 (encoder.cpp) =====================
#include "encoder.h"

volatile long Encoder::count_l = 0;
volatile long Encoder::count_r = 0;

// ---------- 中断服务函数 (IRAM_ATTR 确保在 IRAM 中执行) ----------
// A 相变化时更新计数：A≠B → 正向，A=B → 反向
void IRAM_ATTR encL_ISR() {
    if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B))
        Encoder::count_l++;
    else
        Encoder::count_l--;
}
void IRAM_ATTR encR_ISR() {
    if (digitalRead(ENC_R_A) != digitalRead(ENC_R_B))
        Encoder::count_r++;
    else
        Encoder::count_r--;
}

void Encoder::begin() {
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), encL_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), encR_ISR, CHANGE);
}
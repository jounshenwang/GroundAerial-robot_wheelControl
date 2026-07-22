// ===================== 清洁电机控制模块 (cleaner.cpp) =====================
// TB6612 独立驱动，1 路 PWM 调速 + 方向控制
#include "cleaner.h"
#include <driver/ledc.h>

void Cleaner::begin() {
    // STBY 使能
    pinMode(CLEAN_STBY_PIN, OUTPUT);
    digitalWrite(CLEAN_STBY_PIN, HIGH);

    // 方向由硬件固定：IN1→3.3V, IN2→GND（仅正转）
    // LEDC PWM 配置 (5kHz, 8bit, 与主电机一致)
    ledcSetup(CLEAN_PWM_CH, 5000, 8);
    ledcAttachPin(CLEAN_PWM_PIN, CLEAN_PWM_CH);

    off();
}

void Cleaner::set(bool on) {
    if (on) {
        // 硬件固定 IN1=3.3V, IN2=GND → 正转，PWM 调速
        ledcWrite(CLEAN_PWM_CH, CLEAN_PWM_SPEED);
    } else {
        // 关闭 PWM → 滑行停转
        ledcWrite(CLEAN_PWM_CH, 0);
    }
}

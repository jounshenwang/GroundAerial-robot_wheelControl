// ===================== 电机驱动模块 (motor.cpp) =====================
#include "motor.h"
#include <driver/ledc.h>

// LEDC 通道映射：
//   通道 0 → 左前 (PWMA)
//   通道 1 → 左后 (PWMB)
//   通道 2 → 右后 (PWMC)
//   通道 3 → 右前 (PWMD)

void motorBegin() {
    // STBY 高电平使能驱动芯片
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);

    // 方向引脚
    pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT);
    pinMode(CIN1_PIN, OUTPUT); pinMode(CIN2_PIN, OUTPUT);
    pinMode(DIN1_PIN, OUTPUT); pinMode(DIN2_PIN, OUTPUT);

    // LEDC PWM 配置 (5kHz, 8bit)
    ledcSetup(0, 5000, 8); ledcAttachPin(PWMA_PIN, 0);
    ledcSetup(1, 5000, 8); ledcAttachPin(PWMB_PIN, 1);
    ledcSetup(2, 5000, 8); ledcAttachPin(PWMC_PIN, 2);
    ledcSetup(3, 5000, 8); ledcAttachPin(PWMD_PIN, 3);
}

void motorSet(int out, int p1, int p2, int channel) {
    if (out > 1) {
        // 正向：IN1=H, IN2=L
        digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
    } else if (out < -1) {
        // 反向：IN1=L, IN2=H
        digitalWrite(p1, LOW); digitalWrite(p2, HIGH);
    } else {
        // 死区/释放：IN1=L, IN2=L 自由滑行
        // 不采用短路制动（IN=H），否则 PID 小修正量被强行阻挡，
        // 积分持续累积后突然跳出死区导致电机突跳式震荡。
        digitalWrite(p1, LOW); digitalWrite(p2, LOW);
    }
    ledcWrite(channel, constrain(abs(out), 0, 255));
}

void motorSetDifferential(int left, int right) {
    motorSet(left,  AIN1_PIN, AIN2_PIN, 0);  // 左前
    motorSet(left,  BIN1_PIN, BIN2_PIN, 1);  // 左后
    motorSet(right, DIN1_PIN, DIN2_PIN, 3);  // 右前
    motorSet(right, CIN1_PIN, CIN2_PIN, 2);  // 右后
}

void motorStopAll(bool activeBrake) {
    if (activeBrake) {
        // 防溜坡：IN=H, PWM=255 短路制动
        digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, HIGH);
        digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, HIGH);
        digitalWrite(CIN1_PIN, HIGH); digitalWrite(CIN2_PIN, HIGH);
        digitalWrite(DIN1_PIN, HIGH); digitalWrite(DIN2_PIN, HIGH);
        ledcWrite(0, 255); ledcWrite(1, 255);
        ledcWrite(2, 255); ledcWrite(3, 255);
    } else {
        // 释放：IN=L, PWM=0
        digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, LOW);
        digitalWrite(BIN1_PIN, LOW); digitalWrite(BIN2_PIN, LOW);
        digitalWrite(CIN1_PIN, LOW); digitalWrite(CIN2_PIN, LOW);
        digitalWrite(DIN1_PIN, LOW); digitalWrite(DIN2_PIN, LOW);
        ledcWrite(0, 0); ledcWrite(1, 0);
        ledcWrite(2, 0); ledcWrite(3, 0);
    }
}
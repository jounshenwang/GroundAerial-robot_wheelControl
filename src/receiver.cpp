// ===================== 遥控接收机模块 (receiver.cpp) =====================
#include "receiver.h"

volatile uint32_t Receiver::ch1  = 1500;
volatile uint32_t Receiver::ch2  = 1500;
volatile uint32_t Receiver::ch9  = 1000;
volatile uint32_t Receiver::ch10 = 1000;

// ---------- 中断服务函数 ----------
// 捕捉 PWM 高电平脉宽，换算为 1000–2000 µs 的通道值
void IRAM_ATTR readCh1_ISR() {
    static uint32_t t = 0;
    if (digitalRead(PX4_CH1_PIN)) t = micros();
    else Receiver::ch1 = constrain(micros() - t, 1000, 2000);
}
void IRAM_ATTR readCh2_ISR() {
    static uint32_t t = 0;
    if (digitalRead(PX4_CH2_PIN)) t = micros();
    else Receiver::ch2 = constrain(micros() - t, 1000, 2000);
}
void IRAM_ATTR readCh9_ISR() {
    static uint32_t t = 0;
    if (digitalRead(PX4_CH9_PIN)) t = micros();
    else Receiver::ch9 = constrain(micros() - t, 1000, 2000);
}
void IRAM_ATTR readCh10_ISR() {
    static uint32_t t = 0;
    if (digitalRead(PX4_CH10_PIN)) t = micros();
    else Receiver::ch10 = constrain(micros() - t, 1000, 2000);
}

void Receiver::begin() {
    // 输入上拉，防止浮空引脚产生毛刺中断
    pinMode(PX4_CH1_PIN, INPUT_PULLUP);
    pinMode(PX4_CH2_PIN, INPUT_PULLUP);
    pinMode(PX4_CH9_PIN, INPUT_PULLUP);
    pinMode(PX4_CH10_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PX4_CH1_PIN),  readCh1_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH2_PIN),  readCh2_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH9_PIN),  readCh9_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH10_PIN), readCh10_ISR, CHANGE);
}
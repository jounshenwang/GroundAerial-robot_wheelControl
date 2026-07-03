// ===================== 安全与可靠性模块 (safety.cpp) =====================
#include "safety.h"
#include "encoder.h"
#include "receiver.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "autoclean.h"

#include "esp_task_wdt.h"
#include <cstring>       // memcpy

// ── 全局变量定义 ──
SafetyState Safety::state = SAFETY_INIT;
uint16_t    Safety::faultFlags = FAULT_NONE;

// ── 内部状态变量 ──
static uint32_t lastWdtFeed = 0;

// 信号丢失检测
static uint32_t lastSignalSnapshot = 0;
static uint32_t chSnapshot[4] = {0};
static uint32_t signalFreezeCount = 0;

// 堵转检测
static uint32_t stallStartL = 0;
static uint32_t stallStartR = 0;
static bool     stallWarnL = false;
static bool     stallWarnR = false;

// 急停恢复
static bool     recoverArmed = false;  // false=等待 ch9 拉低准备

// ════════════════════════════════════════════════════════════════
//  上电自检 (POST)
// ════════════════════════════════════════════════════════════════
void Safety::runPOST() {
    Serial.println("═══ Safety POST ═══");
    faultFlags = FAULT_NONE;
    uint8_t errors = 0;

    // ① 接收机信号范围检测
    uint32_t ch[4] = {
        Receiver::ch1, Receiver::ch2,
        Receiver::ch9, Receiver::ch10
    };
    bool rxOK = true;
    for (int i = 0; i < 4; i++) {
        if (ch[i] < 800 || ch[i] > 2200) {
            static const char* chNames[] = {"1", "2", "9", "10"};
            Serial.print("  [FAIL] CH");
            Serial.print(chNames[i]);
            Serial.print(" = ");
            Serial.print(ch[i]);
            Serial.println(" (out of range)");
            rxOK = false;
            errors++;
        }
    }
    if (rxOK) {
        Serial.println("  [OK]  Receiver signals valid");
        memcpy(chSnapshot, ch, sizeof(chSnapshot));
        signalFreezeCount = 0;

        // 预防性检测：全部通道 ≈ 1500 可能接收机未连接
        if (abs((int)ch[0] - 1500) <= 5 &&
            abs((int)ch[1] - 1500) <= 5 &&
            abs((int)ch[2] - 1500) <= 5 &&
            abs((int)ch[3] - 1500) <= 5) {
            Serial.println("  [WARN] All channels ~ 1500 — verify RX connection");
        }
    } else {
        faultFlags |= FAULT_RX_SIGNAL;
    }

    // ② 看门狗初始化 (WDT_TIMEOUT 超时 → 硬件复位)
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    lastWdtFeed = millis();
    Serial.println("  [OK]  Watchdog armed (3s timeout)");

    // ③ 综合判定
    if (errors > 0) {
        triggerEStop("POST failed - RX signal");
    } else {
        state = SAFETY_NORMAL;
        Serial.println("  >>> POST PASSED, entering NORMAL");
    }
    Serial.println("═══ POST End ═══");
}

// ════════════════════════════════════════════════════════════════
//  喂看门狗 (每控制周期无条件调用，独立于安全状态)
// ════════════════════════════════════════════════════════════════
void Safety::feedWatchdog() {
    uint32_t now = millis();
    if (now - lastWdtFeed > 500) {
        esp_task_wdt_reset();
        lastWdtFeed = now;
    }
}

// ════════════════════════════════════════════════════════════════
//  运行时监控 (每控制周期调用)
// ════════════════════════════════════════════════════════════════
void Safety::runtimeMonitor(int speedL, int speedR, int outL, int outR) {
    uint32_t now = millis();

    // 急停中不再重复检测
    if (state == SAFETY_ESTOP) return;

    // ── 2. 接收机信号丢失检测 ──
    // 以 SAFETY_SIGNAL_TIMEOUT/5 = 100ms 为采样间隔，双层检测：
    //   ① 硬边界：任一通道 <800 或 >2200 → 立即判定信号丢失
    //      (接收机掉线后引脚上拉，测量值接近 0 或 3000+)
    //   ② 冻结检测：仅 ch1/ch2（模拟摇杆）连续 20 次采样不变 ~2s → 判定丢失
    //      排除 ch9/ch10（开关通道），开关正常状态就是恒定不变的
    if (now - lastSignalSnapshot > SAFETY_SIGNAL_TIMEOUT / 5) {
        lastSignalSnapshot = now;
        uint32_t cur[4] = {
            Receiver::ch1, Receiver::ch2,
            Receiver::ch9, Receiver::ch10
        };

        // ① 硬边界检测
        bool outOfRange = false;
        for (int i = 0; i < 4; i++) {
            if (cur[i] < 800 || cur[i] > 2200) { outOfRange = true; break; }
        }
        if (outOfRange) {
            faultFlags |= FAULT_RX_SIGNAL;
            triggerEStop("RX signal out of range");
            return;
        }

        // ② 冻结检测：仅对模拟通道 ch1/ch2 检查
        bool frozen = true;
        for (int i = 0; i < 2; i++) {
            if (cur[i] != chSnapshot[i]) { frozen = false; break; }
        }
        if (frozen) {
            signalFreezeCount++;
        } else {
            memcpy(chSnapshot, cur, sizeof(chSnapshot));
            signalFreezeCount = 0;
            faultFlags &= ~FAULT_RX_SIGNAL;
        }
        // 连续 20 次采样不变 (~2s) → 判定信号冻结
        if (signalFreezeCount >= 20) {
            faultFlags |= FAULT_RX_SIGNAL;
            triggerEStop("RX signal frozen (sticks)");
            return;
        }
    }

    // ── 2b. IMU 数据有效性检查 ──
    // 在倾角检测之前检查，避免恢复期间使用过时角度数据导致保护盲区
    if (!IMU::dataValid) {
        faultFlags |= FAULT_IMU;
        triggerEStop("IMU data invalid (I2C error)");
        return;
    } else {
        faultFlags &= ~FAULT_IMU;
    }

    // ── 3. 倾角超限检测 (翻倒保护) ──
    if (fabs(IMU::angle_pitch) > SAFETY_TILT_MAX) {
        faultFlags |= FAULT_OVER_TILT;
        triggerEStop("Over-tilt");
        return;
    } else {
        faultFlags &= ~FAULT_OVER_TILT;
    }

    // ── 4. 堵转检测 ──
    //   PID 输出持续饱和 (≥ SAFETY_STALL_PWM) 且编码器速度近乎零 → 物理堵转
    if (abs(outL) >= SAFETY_STALL_PWM && abs(speedL) < 2) {
        if (!stallWarnL) { stallWarnL = true; stallStartL = now; }
        else if (now - stallStartL > SAFETY_STALL_MS) {
            faultFlags |= FAULT_STALL_L;
            triggerEStop("Left motor stall");
            return;
        }
    } else {
        stallWarnL = false;
        faultFlags &= ~FAULT_STALL_L;
    }

    if (abs(outR) >= SAFETY_STALL_PWM && abs(speedR) < 2) {
        if (!stallWarnR) { stallWarnR = true; stallStartR = now; }
        else if (now - stallStartR > SAFETY_STALL_MS) {
            faultFlags |= FAULT_STALL_R;
            triggerEStop("Right motor stall");
            return;
        }
    } else {
        stallWarnR = false;
        faultFlags &= ~FAULT_STALL_R;
    }

    // ── 5. 状态降级判定 ──
    if (faultFlags & (FAULT_IMU | FAULT_ENCODER_L | FAULT_ENCODER_R)) {
        state = SAFETY_DEGRADED;
    } else if (faultFlags & FAULT_BAT_LOW) {
        state = SAFETY_WARNING;
    } else {
        state = SAFETY_NORMAL;
    }
}

// ════════════════════════════════════════════════════════════════
//  急停
// ════════════════════════════════════════════════════════════════
void Safety::triggerEStop(const char* reason) {
    if (state == SAFETY_ESTOP) return;   // 防止重复触发

    state = SAFETY_ESTOP;
    recoverArmed = false;

    // — 立即切断所有动力 —
    motorStopAll(true);          // 四轮短路制动 (防溜坡)
    digitalWrite(CLEAN_MOTOR_PIN, LOW);
    pidL.reset();
    pidR.reset();
    autoCleanReset();

    Serial.print(">>> SAFETY_ESTOP: ");
    Serial.println(reason);
}

// ════════════════════════════════════════════════════════════════
//  急停恢复
// ════════════════════════════════════════════════════════════════
bool Safety::tryRecover() {
    if (state != SAFETY_ESTOP) return false;

    // 恢复手势：ch9 < 1200 (飞行) → ch9 > 1700 (地面) 切换一次
    if (!recoverArmed) {
        if (Receiver::ch9 < 1200) {
            recoverArmed = true;
            Serial.println(">>> Recover: ch9 low, arm ready");
        }
        return false;
    } else {
        if (Receiver::ch9 > 1700) {
            recoverArmed = false;
            state = SAFETY_NORMAL;
            // 只清除瞬态故障，保留硬件故障位（IMU/编码器/驱动等）
            // 这样如果硬件故障未恢复，runtimeMonitor 会在下一周期重新触发降级
            faultFlags &= ~(FAULT_OVER_TILT | FAULT_STALL_L | FAULT_STALL_R |
                             FAULT_POS_OVERRUN | FAULT_RX_SIGNAL);
            Serial.println(">>> Safety recovered to NORMAL");
            return true;
        }
        return false;
    }
}

// ════════════════════════════════════════════════════════════════
//  查询 / 输出辅助
// ════════════════════════════════════════════════════════════════
bool Safety::hasCriticalFault() {
    // 致命故障：信号丢失 → 完全失控 / 倾角超限 → 翻倒 / 堵转 → 可能烧毁电机
    // FAULT_POS_OVERRUN 是症状类告警，不在此列
    const uint16_t CRITICAL = FAULT_RX_SIGNAL | FAULT_OVER_TILT |
                               FAULT_STALL_L | FAULT_STALL_R;
    return (faultFlags & CRITICAL) != 0;
}

void Safety::printStatus() {
    Serial.print("[Safe] S:");
    Serial.print((int)state);
    Serial.print(" F:0x");
    Serial.print(faultFlags, HEX);

    struct { uint16_t mask; const char* label; } const mappings[] = {
        {FAULT_IMU,         " IMU"},
        {FAULT_ENCODER_L,   " ENC_L"},
        {FAULT_ENCODER_R,   " ENC_R"},
        {FAULT_RX_SIGNAL,   " RX"},
        {FAULT_BAT_LOW,     " BAT"},
        {FAULT_OVER_TILT,   " TILT"},
        {FAULT_STALL_L,     " STALL_L"},
        {FAULT_STALL_R,     " STALL_R"},
        {FAULT_POS_OVERRUN, " POS_OVR"},
    };
    for (auto& m : mappings) {
        if (faultFlags & m.mask) Serial.print(m.label);
    }
    Serial.println();
}

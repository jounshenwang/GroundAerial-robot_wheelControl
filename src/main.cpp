// ===================== V12 模块化主入口 + 安全可靠集成 =====================
// PlatformIO: 编译系统会自动找到 main.cpp 作为入口
#include <Arduino.h>

#include "config.h"
#include "encoder.h"
#include "receiver.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "autoclean.h"
#include "cleaner.h"
#include "safety.h"
#include "comm.h"
#include "mavlink_bridge.h"

// ────────────────────────────────────────────────────────────────
// 全局控制状态 (仅在主控模块内可见)
// ────────────────────────────────────────────────────────────────
static long targetPosL = 0, targetPosR = 0;   // PID 位置目标
static float ff_fade   = 0;                   // 重力前馈淡入因子 0→1
static bool lastGround = false;                // 上一周期模式 (用于沿变检测)
static bool imuOk = false;                     // IMU 初始化成功标志

// ────────────────────────────────────────────────────────────────
// setup()
// ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n======= V12 Modular (4-MOTOR + MPU6050) =======");

    Cleaner::begin();

    bool imuOk = IMU::begin();
    if (!imuOk) {
        Serial.println("⚠ MPU6050 初始化失败，FAULT_IMU 置位");
        Safety::faultFlags |= FAULT_IMU;
    }               // MPU6050 初始化
    Encoder::begin();           // 编码器中断
    Receiver::begin();          // 遥控接收机中断
    motorBegin();               // 电机引脚 + PWM
    Comm::commInit();           // UART1 串口通信初始化（未连接 Jetson 时注释）
    MavlinkBridge::begin();     // UART2 飞控 MAVLink 通信

    // PID 目标与编码器计数对齐
    targetPosL = Encoder::readL();
    targetPosR = Encoder::readR();

    // ═══ 上电自检 (POST) ═══
    // 检查接收机信号、初始化看门狗
    Safety::runPOST();
}

// ────────────────────────────────────────────────────────────────
// loop()
// ────────────────────────────────────────────────────────────────
void loop() {
    // ═══ 0. 定时节拍 ═══
    static uint32_t lastTick = 0;
    static uint32_t lastPrint = 0;
    if (millis() - lastTick < CONTROL_PERIOD) return;
    lastTick = millis();

    Comm::commUpdate();                 // 与 Orin Nano 交换通信帧 (50Hz，未连接 Jetson 时注释)
    MavlinkBridge::update();            // 飞控 MAVLink 通信 (HEARTBEAT + 命令收发)

    Safety::feedWatchdog();             // 无条件喂狗 (独立于安全状态)

    // 快照接收机通道到局部变量 (一次读取，后续统一使用，避免 ISR 跨指令撕裂)
    uint32_t rxCh1 = Receiver::ch1, rxCh2 = Receiver::ch2;
    uint32_t rxCh9 = Receiver::ch9, rxCh10 = Receiver::ch10;

    // ═══ A. 外设控制 (不受急停影响) ═══
    Cleaner::set(rxCh10 > 1600 && Safety::state != SAFETY_ESTOP);

    // 姿态更新 (MPU6050 → 互补滤波)
    uint32_t pitchPWM = 1500;  // 默认值（水平）
    if (imuOk) {
        IMU::update();
        // 从俯仰角计算前馈 PWM 值 (1000~2000, 1500=水平)
        pitchPWM = constrain(1500 +
            (IMU::angle_pitch * PITCH_PWM_GAIN), 1000, 2000);
    }

    // ═══ B. 速度测量 (全模式都需要，放在模式分支前) ═══
    static long lastCntL = 0, lastCntR = 0;
    static bool speedInitd = false;
    long cntL, cntR;
    Encoder::readPair(cntL, cntR);
    if (!speedInitd) {
        lastCntL = cntL;              // 首个周期与编码器对齐，避免速度毛刺
        lastCntR = cntR;
        speedInitd = true;
    }
    int speedL = (int)(cntL - lastCntL);
    int speedR = (int)(cntR - lastCntR);
    lastCntL = cntL;
    lastCntR = cntR;

    int outL = 0, outR = 0;     // PID 输出 (供安全监控使用)

    // ═══ C. 模式判断 ═══
    bool groundMode = (rxCh9 > 1700);

    // ═══ D. 急停处理 ═══
    if (Safety::state == SAFETY_ESTOP) {
        Safety::tryRecover();
        // 电机已由 triggerEStop 制动，此处不再操作
        lastGround = groundMode;  // 保持 lastGround 同步，便于恢复后正确沿变检测
    } else {
        // ═══ D+. 降级模式处理 ═══
        if (Safety::state == SAFETY_DEGRADED) {
            // 编码器/IMU 降级 → 禁用自动清扫，归零前馈
            if (autoState != CLEAN_IDLE) autoCleanReset();
            ff_fade = 0;
        }

        if (groundMode) {

            // ─── D1. 空地切换过渡 ───
            if (!lastGround) {
                targetPosL = cntL;
                targetPosR = cntR;
                pidL.reset();
                pidR.reset();
                ff_fade = 0;
                autoCleanReset();

                // 进入地面模式 → 锁死飞控无刷电机
                // (PX4 disarm 后 IO 协处理器继续转发 RC PWM，
                //   不影响 ESP32 读取遥控信号控制轮子)
                if (!MavlinkBridge::sendArmDisarm(false)) {
                    Serial.println("[MAVLink] WARN: FC disarm failed (not connected?)");
                }
            }
            if (ff_fade < 1.0f) ff_fade += FF_FADE_STEP;

            // ─── D2. 坡度自适应参数 ───
            float slope = constrain(
                abs((int)pitchPWM - 1500) / 500.0f, 0.0f, 1.0f);
            float countsPerRev = (float)(ENCODER_PPR * ENCODER_MULT * GEAR_RATIO);
            float maxCPS       = (MAX_RPM / 60.0f) * countsPerRev;
            float maxCPP       = maxCPS * (CONTROL_PERIOD / 1000.0f);
            float dynMaxV      = maxCPP - slope * (maxCPP * (1.0f - MIN_RPM_FACTOR));
            float dynKpV       = KP_V_FLAT + slope * KP_V_SLOPE;

            // ─── D3. 控制源分流 (手动 / 自主清扫 / ROS2 / 静止) ───
            int moveStep  = 0;
            int steerStep = 0;
            static uint8_t prevCommMode = MODE_MANUAL;
            uint8_t curCommMode = Comm::getMode();

            bool manual = (abs((int)rxCh2 - 1500) > THROTTLE_DEADBAND ||
                           abs((int)rxCh1 - 1500) > STEERING_DEADBAND);

            if (curCommMode == MODE_ROS2_AUTO) {
                // 分支 ①：ROS2 自主导航（最高优先级，不受摇杆影响）
                // 速度指令直接来自 Orin Nano 串口，位置控制由 Nav2 完成
                autoCleanReset();
                // 首次进入 ROS2 模式时复位 PID，消除上一模式的残余积分
                if (prevCommMode != MODE_ROS2_AUTO) {
                    pidL.reset();
                    pidR.reset();
                }
            } else if (manual && rxCh2 > 900 && rxCh2 < 2100) {
                // 分支 ②：手动遥控
                moveStep  = map((int)rxCh2, 1000, 2000,
                    THROTTLE_STEP_MIN, THROTTLE_STEP_MAX);
                steerStep = map((int)rxCh1, 1000, 2000,
                    -STEERING_STEP_MIN, -STEERING_STEP_MAX);
                autoCleanReset();
            } else if (rxCh10 > 1600) {
                // 分支 ③：自主"弓"字形清扫
                if (autoState == CLEAN_IDLE) {
                    rowCount = 0;
                    changeAutoState(LINE_FORWARD,
                        cntL, cntR);
                }
                AutoCleanResult r = autoCleanUpdate(
                    cntL, cntR);
                moveStep  = r.moveStep;
                steerStep = r.steerStep;
            } else {
                // 分支 ④：静止驻车
                autoCleanReset();
            }

            // ── D3a. 模式切换过渡处理 ──
            // 退出 ROS2 模式后需重新对齐位置目标，避免位置环阶跃
            if (prevCommMode == MODE_ROS2_AUTO && curCommMode != MODE_ROS2_AUTO) {
                targetPosL = cntL;
                targetPosR = cntR;
                pidL.reset();
                pidR.reset();
            }
            prevCommMode = curCommMode;

            // ─── 分流：ROS2 模式走纯速度环，其他模式走位置环+速度环 ───
            if (curCommMode == MODE_ROS2_AUTO) {
                // ═══ D4-R. 纯速度环控制 (ROS2 模式) ═══
                float feedFwd = ((float)pitchPWM - 1500.0f) / 500.0f
                              * FF_MAP_MAX * ff_fade;
                int vTargetL = Comm::getTargetVelocityL();
                int vTargetR = Comm::getTargetVelocityR();
                vTargetL = constrain(vTargetL, -(int)dynMaxV, (int)dynMaxV);
                vTargetR = constrain(vTargetR, -(int)dynMaxV, (int)dynMaxV);
                outL = pidL.computeVelocity(vTargetL, speedL, dynKpV, feedFwd);
                outR = pidR.computeVelocity(vTargetR, speedR, dynKpV, feedFwd);
            } else {
                // ═══ D4. 位置环目标更新 + 硬限幅 ═══
                targetPosL += (moveStep + steerStep);
                targetPosR += (moveStep - steerStep);
                targetPosL = constrain(targetPosL,
                    cntL - ERROR_LIMIT, cntL + ERROR_LIMIT);
                targetPosR = constrain(targetPosR,
                    cntR - ERROR_LIMIT, cntR + ERROR_LIMIT);

                // ═══ D5. 位置误差超限检测 (带滞回保持) ═══
                // 置位后至少持续 10 个控制周期 (200ms) 才清除，避免瞬态误报
                static uint8_t posOvrCount = 0;
                if (abs(targetPosL - cntL) > ERROR_LIMIT * POS_OVERRUN_THRESHOLD ||
                    abs(targetPosR - cntR) > ERROR_LIMIT * POS_OVERRUN_THRESHOLD) {
                    Safety::faultFlags |= FAULT_POS_OVERRUN;
                    posOvrCount = 0;  // 持续超限，保持置位
                } else if (posOvrCount < 10) {
                    posOvrCount++;    // 等待连续 10 周期正常后才清除
                } else {
                    Safety::faultFlags &= ~FAULT_POS_OVERRUN;
                }

                // ═══ D6. 串级 PID (位置环 + 速度环) ═══
                // 浮点映射替代 map() 整数运算，避免阶梯状输出
                // pitchPWM [1000,2000] → 前馈 [-45,45]，中点 1500 → 0
                float feedFwd = ((float)pitchPWM - 1500.0f) / 500.0f
                              * FF_MAP_MAX * ff_fade;
                outL = pidL.compute(targetPosL, cntL,
                                        speedL, dynMaxV, dynKpV, feedFwd);
                outR = pidR.compute(targetPosR, cntR,
                                        speedR, dynMaxV, dynKpV, feedFwd);
            }

            // ═══ D7. 电机输出 (两种模式共用) ═══
            motorSetDifferential(outL, outR);

        } else {
            // ═══ E. 飞行模式 ═══
            // 刚从地面切回飞行 → 恢复飞控武装
            if (lastGround) {
                if (!MavlinkBridge::sendArmDisarm(true)) {
                    Serial.println("[MAVLink] WARN: FC arm failed (not connected?)");
                }
            }

            targetPosL = cntL;
            targetPosR = cntR;
            motorStopAll(false);          // 完全释放
            pidL.reset();
            pidR.reset();
            ff_fade = 0;
            autoCleanReset();
        }

        lastGround = groundMode;

        // ═══ F. 运行时安全监控 ═══
        Safety::runtimeMonitor(speedL, speedR, outL, outR);
    }

    // ═══ G. 调试输出 ═══
    if (millis() - lastPrint > DEBUG_PRINT_INTERVAL) {
        Serial.print("[Angle]");   Serial.print(IMU::angle_pitch, 1);
        Serial.print("° [Mode]");  Serial.print(groundMode ? "G" : "F");
        Serial.print(" [Safe]");   Serial.print((int)Safety::state);
        Serial.print(" [CH]");
        Serial.print(rxCh1); Serial.print("/");
        Serial.print(rxCh2); Serial.print("/");
        Serial.print(rxCh9); Serial.print("/");
        Serial.print(rxCh10);
        Serial.print(" [Out]");    Serial.print(outL);
        Serial.print("/");         Serial.print(outR);
        Serial.print(" [ff]");     Serial.print(ff_fade, 2);
        Serial.print(" [Auto]");   Serial.print((int)autoState);
        Serial.print(" [Rows]");   Serial.print(rowCount);
        Serial.print("/");         Serial.println(MAX_ROWS);
        Safety::printStatus();
        lastPrint = millis();
    }
}

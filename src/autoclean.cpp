// ===================== 自主"弓"字形清扫状态机 (autoclean.cpp) =====================
#include "autoclean.h"
#include "safety.h"

AutoCleanState autoState = CLEAN_IDLE;
int rowCount = 0;

// 各阶段起始编码器位置 (在 changeAutoState 时锁定)
static long stateStartL = 0;
static long stateStartR = 0;
static uint32_t phaseStartTime = 0;   // 阶段开始时间戳 (超时保护)

void changeAutoState(AutoCleanState newState, long countL, long countR) {
    autoState    = newState;
    stateStartL  = countL;
    stateStartR  = countR;
    phaseStartTime = millis();    // 记录阶段起始时刻 (用于超时检测)
    Serial.print(">>> AutoState → ");
    Serial.println((int)newState);
}

void autoCleanReset() {
    autoState = CLEAN_IDLE;
    rowCount = 0;
}

AutoCleanResult autoCleanUpdate(long countL, long countR) {
    // 未激活 → 返回空指令
    if (autoState == CLEAN_IDLE || autoState == CLEAN_DONE) {
        return {0, 0, (autoState == CLEAN_DONE)};
    }

    // ═══ 阶段超时保护：单阶段执行超过 SAFETY_PHASE_TIMEOUT → 强制复位 ═══
    if (millis() - phaseStartTime > SAFETY_PHASE_TIMEOUT) {
        Serial.print("⚠ AutoClean phase timeout (");
        Serial.print(millis() - phaseStartTime);
        Serial.println("ms), resetting");
        Safety::faultFlags |= FAULT_POS_OVERRUN;
        autoCleanReset();
        return {0, 0, false};
    }

    long progL = countL - stateStartL;
    long progR = countR - stateStartR;

    int move  = 0;
    int steer = 0;

    switch (autoState) {
        case LINE_FORWARD:  // ═══ 1. 正向直行 (上坡) ═══
            move = AUTO_SPEED;
            if ((progL + progR) / 2 >= AUTO_LINE_PULSES) {
                rowCount++;
                if (rowCount >= MAX_ROWS)
                    changeAutoState(CLEAN_DONE, countL, countR);
                else
                    changeAutoState(TURN_RIGHT_1, countL, countR);
            }
            break;

        case TURN_RIGHT_1:  // ═══ 2. 第1次右转90° ═══
            steer = AUTO_TURN_SPEED;  // 左前右后
            if (progL >= AUTO_TURN_PULSES)
                changeAutoState(SHIFT_RIGHT, countL, countR);
            break;

        case SHIFT_RIGHT:   // ═══ 3. 向右横移换行 ═══
            move = AUTO_SPEED;
            if ((progL + progR) / 2 >= AUTO_ROW_PULSES)
                changeAutoState(TURN_RIGHT_2, countL, countR);
            break;

        case TURN_RIGHT_2:  // ═══ 4. 第2次右转90° (面朝下坡) ═══
            steer = AUTO_TURN_SPEED;
            if (progL >= AUTO_TURN_PULSES)
                changeAutoState(LINE_BACKWARD, countL, countR);
            break;

        case LINE_BACKWARD: // ═══ 5. 反向直行 (下坡) ═══
            move = AUTO_SPEED;
            if ((progL + progR) / 2 >= AUTO_LINE_PULSES) {
                rowCount++;
                if (rowCount >= MAX_ROWS)
                    changeAutoState(CLEAN_DONE, countL, countR);
                else
                    changeAutoState(TURN_LEFT_1, countL, countR);
            }
            break;

        case TURN_LEFT_1:   // ═══ 6. 第1次左转90° ═══
            steer = -AUTO_TURN_SPEED;  // 右前左后
            if (progR >= AUTO_TURN_PULSES)
                changeAutoState(SHIFT_LEFT, countL, countR);
            break;

        case SHIFT_LEFT:    // ═══ 7. 向左横移换行 ═══
            move = AUTO_SPEED;
            if ((progL + progR) / 2 >= AUTO_ROW_PULSES)
                changeAutoState(TURN_LEFT_2, countL, countR);
            break;

        case TURN_LEFT_2:   // ═══ 8. 第2次左转90° (面朝上坡) ═══
            steer = -AUTO_TURN_SPEED;
            if (progR >= AUTO_TURN_PULSES)
                changeAutoState(LINE_FORWARD, countL, countR);
            break;

        default:
            break;
    }

    return {move, steer, true};
}
// ===================== MPU6050 姿态传感器 (imu.cpp) =====================
#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

float IMU::angle_pitch = 0.0;
bool IMU::dataValid = false;

// I2C 恢复状态机：非阻塞方式，跨多个控制周期逐步完成恢复
enum I2cRecoveryPhase : uint8_t {
    I2C_OK = 0,       // 正常
    I2C_RESETTING = 1,// Wire.end() + Wire.begin() 完成，等待稳定
    I2C_WAKING = 2,   // 写入唤醒寄存器，等待传感器就绪
};
static I2cRecoveryPhase i2cRecovery = I2C_OK;
static uint8_t i2cErrorCount = 0;
static uint32_t i2cStepStart = 0;  // 当前恢复步骤的开始时间戳
static bool   angleInitialized = false;  // 角度首次初始化标志 (I2C 恢复时重置)
static const uint8_t I2C_RETRY_THRESH = 10;

/// 读取 WHO_AM_I 寄存器校验连接
static uint8_t readWhoAmI() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);
    if (Wire.endTransmission() != 0) return 0x00;
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return (Wire.available() >= 1) ? Wire.read() : 0x00;
}

bool IMU::begin() {
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);

    // 唤醒 MPU6050 (PWR_MGMT_1 寄存器写 0)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    if (Wire.endTransmission() == 0) {
        uint8_t whoami = readWhoAmI();
        Serial.print("✓ MPU6050 WHO_AM_I = 0x");
        Serial.println(whoami, HEX);
        dataValid = true;
        return true;
    }
    Serial.println("✗ MPU6050 连接失败");
    return false;
}

void IMU::update() {
    // ── I2C 总线恢复状态机 (非阻塞) ──
    if (i2cRecovery == I2C_RESETTING) {
        // 等待稳定时间到达
        if (millis() - i2cStepStart < 60) return;  // 总共等待约 60ms
        // 重新初始化 I2C + 唤醒 MPU6050
        Wire.end();
        Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x6B);
        Wire.write(0);
        Wire.endTransmission();
        i2cRecovery = I2C_WAKING;
        i2cStepStart = millis();
        // 跳过本周期读取
        i2cErrorCount = 0;
        return;
    }
    if (i2cRecovery == I2C_WAKING) {
        // 等待唤醒稳定后再尝试读取
        if (millis() - i2cStepStart < 50) return;
        i2cRecovery = I2C_OK;
        Serial.println("⚠ IMU I2C recovered");
        // 标记角度需要从加速度计重新初始化（排除恢复前的陀螺漂移）
        angleInitialized = false;
        // 继续执行下面的读取，若仍失败会重新累积错误
    }

    // 发起读取：从 0x3B 开始读 14 字节 (ACCEL+温度+GYRO)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);

    if (Wire.available() < 14) {
        i2cErrorCount++;
        if (i2cErrorCount > I2C_RETRY_THRESH &&
            i2cRecovery == I2C_OK) {
            Serial.println("⚠ IMU I2C error, resetting bus (async)...");
            dataValid = false;              // 标记 IMU 数据无效，安全模块应触发保护
            i2cRecovery = I2C_RESETTING;
            i2cStepStart = millis();
        }
        return;
    }
    i2cErrorCount = 0;  // 读取成功 → 清零错误计数

    // 注意：必须先读入局部变量再组合，避免 C++ 未定义求值顺序
    uint8_t ax_h = Wire.read(), ax_l = Wire.read();
    uint8_t ay_h = Wire.read(), ay_l = Wire.read();
    uint8_t az_h = Wire.read(), az_l = Wire.read();
    Wire.read(); Wire.read();  // 跳过温度
    uint8_t gx_h = Wire.read(), gx_l = Wire.read();
    uint8_t gy_h = Wire.read(), gy_l = Wire.read();
    uint8_t gz_h = Wire.read(), gz_l = Wire.read();

    int16_t ax = (int16_t)((uint16_t)ax_h << 8 | ax_l);
    int16_t ay = (int16_t)((uint16_t)ay_h << 8 | ay_l);
    int16_t az = (int16_t)((uint16_t)az_h << 8 | az_l);
    int16_t gx = (int16_t)((uint16_t)gx_h << 8 | gx_l);
    int16_t gy = (int16_t)((uint16_t)gy_h << 8 | gy_l);
    int16_t gz = (int16_t)((uint16_t)gz_h << 8 | gz_l);

    // 加速度计俯仰角 (度)
    float accel_pitch = atan2((float)ay,
         sqrt((float)ax * ax + (float)az * az)) * 180.0 / PI;

    // 陀螺仪俯仰角速度 (度/秒)
    float gyro_rate = (float)gy / 131.0;

    // ═══ 互补滤波 (基于实际 dt) ═══
    // 时间常数 tau ≈ 1 秒; alpha = tau / (tau + dt)
    // 短 dt → 更依赖陀螺仪; 长 dt → 更信任加速度计
    static uint32_t lastUpdateUs = 0;
    uint32_t now = micros();
    float dt = constrain((now - lastUpdateUs) * 1e-6f, 0.001f, 0.1f);
    lastUpdateUs = now;

    // 首次运行或 I2C 恢复后 — 从加速度计初始化角度（排除陀螺仪漂移）
    if (!angleInitialized) {
        angle_pitch = accel_pitch;
        angleInitialized = true;
        return;
    }

    const float tau = 1.0f;
    float alpha = constrain(dt / (tau + dt), 0.01f, 0.99f);
    angle_pitch = (1.0f - alpha) * (angle_pitch + gyro_rate * dt)
                + alpha * accel_pitch;

    dataValid = true;  // 成功完成一次完整读取+滤波，标记数据有效
}
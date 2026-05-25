// ===================== ESP32-S3 陆空两栖机器人 V9.2 纯净无ToF版 =====================
#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h> // 仅保留标准 I2C 库

// ------------------- [1. 硬件引脚精确映射] -------------------
// 接收机通道
#define PX4_CH1_PIN  13   // 右手左右摇杆 (对应底盘转向)
#define PX4_CH2_PIN  12   // 右手前后摇杆 (对应底盘前进/后退)
#define PX4_CH9_PIN  11   // 模式切换开关 (低电平飞行，高电平陆地)
#define PX4_CH10_PIN 10   // 清洁电机物理开关

// 底盘电机驱动引脚
#define STBY_PIN      4
#define PWMA_PIN      5
#define AIN1_PIN      7
#define AIN2_PIN      6
#define PWMD_PIN      38
#define DIN1_PIN      37
#define DIN2_PIN      39

// 清洁电机控制引脚
#define CLEAN_MOTOR_PIN 47 

// 霍尔编码器引脚
#define ENC_L_A      15
#define ENC_L_B      16
#define ENC_R_A      36
#define ENC_R_B      35

// I2C 引脚定义（专用于 MPU6050）
#define MPU_SDA_PIN   8   
#define MPU_SCL_PIN   3   
#define MPU6050_ADDR  0x68

// ------------------- [2. 常量与控制参数] -------------------
#define ERROR_LIMIT    800   // 位置环误差保护限幅 (单位：脉冲)
#define I_V_LIMIT      100   // 速度环积分限幅
#define CONTROL_PERIOD 20    // 控制周期 20ms (50Hz)
#define DEBUG_PRINT_INTERVAL 1000 // 串口输出间隔 1000ms

// ------------------- [3. 核心数据结构与全局变量] -------------------
// 串级 PID 结构体
struct CascadePID {
    float kp_p = 1.2, ki_p = 0.01, p_integral = 0;
    float kp_v = 8.5, ki_v = 1.2, kd_v = 0.5, v_integral = 0, v_last_err = 0;
};
CascadePID pidL, pidR;

volatile long count_l = 0, count_r = 0; 
long targetPosL = 0, targetPosR = 0;    
volatile uint32_t ch1=1500, ch2=1500, ch9=1000, ch10=1000;
volatile uint32_t pitch=1500; // 姿态前馈变量

// 互补滤波姿态解算变量
float angle_pitch = 0.0; 
bool angle_initialized = false;
bool last_mode_ground = false; 
float ff_fade_factor = 0; 

uint8_t readMPU6050WhoAmI() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);
    if (Wire.endTransmission() != 0) return 0x00;
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return 0x00;
    return Wire.read();
}

// ------------------- [4. 中断服务函数 (ISR)] -------------------
void IRAM_ATTR encL() { if(digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) count_l++; else count_l--; }
void IRAM_ATTR encR() { if(digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) count_r++; else count_r--; }

void readCh1() { static uint32_t t=0; if(digitalRead(PX4_CH1_PIN)) t=micros(); else ch1=constrain(micros()-t,1000,2000); }
void readCh2() { static uint32_t t=0; if(digitalRead(PX4_CH2_PIN)) t=micros(); else ch2=constrain(micros()-t,1000,2000); }
void readCh9() { static uint32_t t=0; if(digitalRead(PX4_CH9_PIN)) t=micros(); else ch9=constrain(micros()-t,1000,2000); }
void readCh10(){ static uint32_t t=0; if(digitalRead(PX4_CH10_PIN)) t=micros(); else ch10=constrain(micros()-t,1000,2000); }

// ------------------- [5. 电机控制与 PID 算法] -------------------
void setMotor(int out, int p1, int p2, int channel) {
    if (out > 5) {
        digitalWrite(p1, HIGH);
        digitalWrite(p2, LOW);
    } else if (out < -5) {
        digitalWrite(p1, LOW);
        digitalWrite(p2, HIGH);
    } else {
        digitalWrite(p1, HIGH); 
        digitalWrite(p2, HIGH);
    }
    ledcWrite(channel, constrain(abs(out), 0, 255));
}

int computeCascadeFF(CascadePID &p, long target, long actualPos, int actualSpeed, float max_v, float kp_v) {
    float p_err = (float)(target - actualPos);
    p.p_integral = constrain(p.p_integral + p_err, -50, 50);
    float v_target = constrain(p.kp_p * p_err + p.ki_p * p.p_integral, -max_v, max_v); 

    float v_err = v_target - (float)actualSpeed;
    p.v_integral = constrain(p.v_integral + v_err, -I_V_LIMIT, I_V_LIMIT);
    float pid_output = kp_v * v_err + p.ki_v * p.v_integral + p.kd_v * (v_err - p.v_last_err);
    p.v_last_err = v_err;

    // 地形自适应重力前馈
    float gravity_ff = map((int)pitch, 1000, 2000, -45, 45); 
    float final_output = pid_output + (gravity_ff * ff_fade_factor);

    return (int)constrain(final_output, -255, 255);
}

// ------------------- [6. 系统初始化 setup] -------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n======= V9.2 START (ToF REMOVED, MPU6050 ONLY) =======");

    // 初始化标准 Wire I2C 总线，分配给 MPU6050
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, 400000);
    
    // 唤醒 MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 寄存器
    Wire.write(0);    // 写入 0 唤醒
    if (Wire.endTransmission() == 0) {
        uint8_t whoami = readMPU6050WhoAmI();
        Serial.print("✓ MPU6050 Initialized, WHO_AM_I=0x");
        Serial.println(whoami, HEX);
        if (whoami != 0x68) {
            Serial.println("✗ MPU6050 返回 WHO_AM_I 非 0x68，请检查模块地址或连接。");
        }
    } else {
        Serial.println("✗ MPU6050 Connection FAILED! Please check wiring.");
    }

    // 初始化电机与驱动引脚
    pinMode(CLEAN_MOTOR_PIN, OUTPUT);
    digitalWrite(CLEAN_MOTOR_PIN, LOW);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(DIN1_PIN, OUTPUT);
    pinMode(DIN2_PIN, OUTPUT);

    ledcSetup(0, 5000, 8);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(PWMA_PIN, 0);
    ledcAttachPin(PWMD_PIN, 1);

    // 绑定外部中断
    attachInterrupt(digitalPinToInterrupt(PX4_CH1_PIN), readCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH2_PIN), readCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH9_PIN), readCh9, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH10_PIN), readCh10, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR, CHANGE);
}

// ------------------- [7. 主控制循环] -------------------
void loop() {
    static uint32_t lastSampleTime = 0;
    if (millis() - lastSampleTime < CONTROL_PERIOD) return;
    lastSampleTime = millis();

    // ================= [A. 独立外设控制与数据获取] =================
    // 清洁电机开关
    if (ch10 > 1600) digitalWrite(CLEAN_MOTOR_PIN, HIGH);
    else digitalWrite(CLEAN_MOTOR_PIN, LOW);

    // 高速读取 MPU6050 原始数据
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // 加速度计 X 轴高位寄存器
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true); 
    
    if (Wire.available() >= 14) {
        int16_t ax = Wire.read() << 8 | Wire.read();
        int16_t ay = Wire.read() << 8 | Wire.read();
        int16_t az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // 跳过温度数据 (2字节)
        int16_t gx = Wire.read() << 8 | Wire.read();
        int16_t gy = Wire.read() << 8 | Wire.read();
        int16_t gz = Wire.read() << 8 | Wire.read();

        float accel_pitch = atan2((float)ay, sqrt((float)ax*ax + (float)az*az)) * 180.0 / PI;
        float gyro_pitch_rate = (float)gy / 131.0; 

        if (!angle_initialized) {
            angle_pitch = accel_pitch;
            angle_initialized = true;
        }
        angle_pitch = 0.98 * (angle_pitch + gyro_pitch_rate * 0.02) + 0.02 * accel_pitch;
        pitch = constrain(1500 + (angle_pitch * (500.0 / 60.0)), 1000, 2000);

        Serial.print("RAW ax="); Serial.print(ax);
        Serial.print(" ay="); Serial.print(ay);
        Serial.print(" az="); Serial.print(az);
        Serial.print(" gy="); Serial.print(gy);
        Serial.print(" gyro_pitch_rate="); Serial.print(gyro_pitch_rate, 2);
        Serial.print(" accel_pitch="); Serial.print(accel_pitch, 2);
        Serial.print(" angle_pitch="); Serial.print(angle_pitch, 2);
        Serial.println();
    } else {
        Serial.println("✗ MPU6050 read failed: insufficient I2C data.");
    }

    // ================= [B. 陆地模式状态机] =================
    bool current_mode_ground = (ch9 > 1700);

    if (current_mode_ground) {
        if (!last_mode_ground) {
            targetPosL = count_l;
            targetPosR = count_r;
            pidL.p_integral = 0; pidL.v_integral = 0;
            pidR.p_integral = 0; pidR.v_integral = 0;
            ff_fade_factor = 0;
        }
        
        if (ff_fade_factor < 1.0) ff_fade_factor += 0.02; // 动力淡入

        // 根据坡度自适应调整 PID 动态参数
        float slope_factor = constrain(abs((int)pitch - 1500) / 500.0, 0.0, 1.0);
        float dynamic_max_v = 150 - (slope_factor * 80); 
        float dynamic_kp_v = 8.5 + (slope_factor * 4.0); 

        static long last_count_l = 0, last_count_r = 0;
        int act_speed_l = (int)(count_l - last_count_l);
        int act_speed_r = (int)(count_r - last_count_r);
        last_count_l = count_l; last_count_r = count_r;

        int move_step = 0, steer_step = 0;
        if (ch2 > 900 && ch2 < 2100) {
            if (abs((int)ch2 - 1500) > 80) move_step = map((int)ch2, 1000, 2000, -50, 50);
            if (abs((int)ch1 - 1500) > 50) steer_step = map((int)ch1, 1000, 2000, 25, -25); 
        }


        // 位置累加与硬限幅保护
        targetPosL += (move_step + steer_step);
        targetPosR += (move_step - steer_step);
        targetPosL = constrain(targetPosL, count_l - ERROR_LIMIT, count_l + ERROR_LIMIT);
        targetPosR = constrain(targetPosR, count_r - ERROR_LIMIT, count_r + ERROR_LIMIT);

        int outL = computeCascadeFF(pidL, targetPosL, count_l, act_speed_l, dynamic_max_v, dynamic_kp_v);
        int outR = computeCascadeFF(pidR, targetPosR, count_r, act_speed_r, dynamic_max_v, dynamic_kp_v);

        setMotor(outL, AIN1_PIN, AIN2_PIN, 0);
        setMotor(outR, DIN1_PIN, DIN2_PIN, 1);

    } else {
        // ================= [C. 飞行模式] =================
        targetPosL = count_l;
        targetPosR = count_r;
        setMotor(0, AIN1_PIN, AIN2_PIN, 0);
        setMotor(0, DIN1_PIN, DIN2_PIN, 1);
        ff_fade_factor = 0;
        // 保留 angle_pitch 值，以便串口输出显示实际 MPU6050 角度
    }

    last_mode_ground = current_mode_ground;

    // ================= [D. 串口监视器] =================
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > DEBUG_PRINT_INTERVAL) {
        Serial.print("[Angle]"); Serial.print(angle_pitch, 1); 
        Serial.print("° [MappedPitch]"); Serial.print(pitch);
        Serial.print(" [CH1]"); Serial.print(ch1);
        Serial.print(" [CH2]"); Serial.print(ch2);
        Serial.print(" [Mode]"); Serial.println(current_mode_ground ? "Ground" : "Flight");
        lastPrintTime = millis();
    }
}
// ===================== ESP32-S3 陆空两栖机器人 V8 PRO (右手控制+防跌落终极版) =====================
#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <VL53L1X.h>

// ------------------- [1. 硬件引脚精确映射] -------------------
// 接收机通道 (飞控/接收机输入，使用右手摇杆隔离飞行油门)
#define PX4_CH1_PIN  13   // 右手左右摇杆 (对应底盘转向)
#define PX4_CH2_PIN  12   // 右手前后摇杆 (对应底盘前进/后退)
#define PX4_CH9_PIN  11   // 模式切换开关 (低电平飞行，高电平陆地)
#define PX4_CH10_PIN 10   // 清洁电机物理开关 (独立通道)
#define PX4_PITCH_PIN 9    // 地形自适应数据 (飞控 Pitch 映射)

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

// 【修改点】：定义双独立总线引脚，不再使用地址切换
#define TOF_SDA_F     8   
#define TOF_SCL_F     3   
#define TOF_SDA_R     17  
#define TOF_SCL_R     18  

// ------------------- [2. 常量与控制参数] -------------------
#define ERROR_LIMIT    800   // 位置环误差保护限幅 (单位：脉冲)
#define I_V_LIMIT      100   // 速度环积分限幅
#define EDGE_DIST_MM   180   // 悬崖/边缘判定距离 (毫米)
#define CONTROL_PERIOD 20    // 控制周期 20ms (50Hz)

// ------------------- [3. 核心数据结构与全局变量] -------------------
TwoWire i2cFront = TwoWire(0);
TwoWire i2cRear = TwoWire(1);
VL53L1X sensorF, sensorR;
uint16_t distF = 0, distR = 0; // 缓存非阻塞距离数据

// 串级 PID 结构体
struct CascadePID {
    float kp_p = 1.2, ki_p = 0.01, p_integral = 0;
    float kp_v = 8.5, ki_v = 1.2, kd_v = 0.5, v_integral = 0, v_last_err = 0;
};
CascadePID pidL, pidR;

volatile long count_l = 0, count_r = 0; 
long targetPosL = 0, targetPosR = 0;    
volatile uint32_t ch1=1500, ch2=1500, ch9=1000, ch10=1000, pitch=1500;

bool last_mode_ground = false; 
float ff_fade_factor = 0; // 模式切换时的动力淡入系数

// ------------------- [4. 中断服务函数 (ISR)] -------------------
void IRAM_ATTR encL() { if(digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) count_l++; else count_l--; }
void IRAM_ATTR encR() { if(digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) count_r++; else count_r--; }

void readCh1() { static uint32_t t=0; if(digitalRead(PX4_CH1_PIN)) t=micros(); else ch1=constrain(micros()-t,1000,2000); }
void readCh2() { static uint32_t t=0; if(digitalRead(PX4_CH2_PIN)) t=micros(); else ch2=constrain(micros()-t,1000,2000); }
void readCh9() { static uint32_t t=0; if(digitalRead(PX4_CH9_PIN)) t=micros(); else ch9=constrain(micros()-t,1000,2000); }
void readCh10(){ static uint32_t t=0; if(digitalRead(PX4_CH10_PIN)) t=micros(); else ch10=constrain(micros()-t,1000,2000); }
void readPitch(){ static uint32_t t=0; if(digitalRead(PX4_PITCH_PIN)) t=micros(); else pitch=constrain(micros()-t,1000,2000); }

void scanI2CBus(TwoWire &wire, const char *name) {
    Serial.print("Scanning "); Serial.print(name); Serial.println(" I2C bus...");
    for (uint8_t addr = 8; addr < 120; addr++) {
        wire.beginTransmission(addr);
        if (wire.endTransmission() == 0) {
            Serial.print("  Found device at 0x");
            if (addr < 16) Serial.print('0');
            Serial.println(addr, HEX);
        }
    }
}

// ------------------- [5. 电机控制与 PID 算法] -------------------
void setMotor(int out, int p1, int p2, int channel) {
    if (out > 5) {
        digitalWrite(p1, HIGH);
        digitalWrite(p2, LOW);
    } else if (out < -5) {
        digitalWrite(p1, LOW);
        digitalWrite(p2, HIGH);
    } else {
        digitalWrite(p1, HIGH); // 电子刹车：双脚制动
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

    float gravity_ff = map((int)pitch, 1000, 2000, -45, 45); 
    float final_output = pid_output + (gravity_ff * ff_fade_factor);

    return (int)constrain(final_output, -255, 255);
}

// ------------------- [6. 系统初始化 setup] -------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n======= V8 PRO START (DUAL-BUS) =======");

    // 【修改点】：初始化第一路 I2C (前)
    i2cFront.begin(TOF_SDA_F, TOF_SCL_F, 400000);
    i2cFront.setClock(400000);
    sensorF.setBus(&i2cFront);
    if (sensorF.init()) {
        sensorF.setTimeout(500);
        sensorF.startContinuous(50);
        Serial.println("✓ VL53L1X Front OK (Pin 8/3)");
    } else {
        Serial.println("✗ Front Fail! Check wiring.");
    }

    // 【修改点】：初始化第二路 I2C (后)
    i2cRear.begin(TOF_SDA_R, TOF_SCL_R, 400000);
    i2cRear.setClock(400000);
    sensorR.setBus(&i2cRear);
    if (sensorR.init()) {
        sensorR.setTimeout(500);
        sensorR.startContinuous(50);
        Serial.println("✓ VL53L1X Rear OK (Pin 17/18)");
    } else {
        Serial.println("✗ Rear Fail! Check wiring.");
    }

    scanI2CBus(i2cFront, "Front");
    scanI2CBus(i2cRear, "Rear");

    pinMode(CLEAN_MOTOR_PIN, OUTPUT);
    digitalWrite(CLEAN_MOTOR_PIN, LOW);

    // 【核心修复】：移除原代码中对 17, 18 的 pinMode(OUTPUT) 操作
    // 这行注释保留，提醒不要恢复原代码的这部分内容
    // pinMode(17, OUTPUT); digitalWrite(17, HIGH); <- 这会导致 Error -1

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

    attachInterrupt(digitalPinToInterrupt(PX4_CH1_PIN), readCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH2_PIN), readCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH9_PIN), readCh9, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_CH10_PIN), readCh10, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PX4_PITCH_PIN), readPitch, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR, CHANGE);
}

// ------------------- [7. 主控制循环] -------------------
void loop() {
    static uint32_t lastSampleTime = 0;
    if (millis() - lastSampleTime < CONTROL_PERIOD) return;
    lastSampleTime = millis();

    // 清洁电机控制 (原封不动)
    if (ch10 > 1600) digitalWrite(CLEAN_MOTOR_PIN, HIGH);
    else digitalWrite(CLEAN_MOTOR_PIN, LOW);

    // 获取测距数据 (非阻塞)
    if (sensorF.dataReady()) distF = sensorF.read();
    if (sensorR.dataReady()) distR = sensorR.read();

    bool current_mode_ground = (ch9 > 1700);

    if (current_mode_ground) {
        if (!last_mode_ground) {
            targetPosL = count_l;
            targetPosR = count_r;
            pidL.p_integral = 0; pidL.v_integral = 0;
            pidR.p_integral = 0; pidR.v_integral = 0;
            ff_fade_factor = 0;
        }
        
        if (ff_fade_factor < 1.0) ff_fade_factor += 0.02;

        // 地形参数计算 (原封不动)
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

        // --- 【修改点】：边缘锁定逻辑 ---
        if (distF > EDGE_DIST_MM && move_step > 0) move_step = 0; // 前方边缘锁死前进
        if (distR > EDGE_DIST_MM && move_step < 0) move_step = 0; // 后方边缘锁死后退

        // 位置控制逻辑 (原封不动)
        targetPosL += (move_step + steer_step);
        targetPosR += (move_step - steer_step);
        targetPosL = constrain(targetPosL, count_l - ERROR_LIMIT, count_l + ERROR_LIMIT);
        targetPosR = constrain(targetPosR, count_r - ERROR_LIMIT, count_r + ERROR_LIMIT);

        int outL = computeCascadeFF(pidL, targetPosL, count_l, act_speed_l, dynamic_max_v, dynamic_kp_v);
        int outR = computeCascadeFF(pidR, targetPosR, count_r, act_speed_r, dynamic_max_v, dynamic_kp_v);

        setMotor(outL, AIN1_PIN, AIN2_PIN, 0);
        setMotor(outR, DIN1_PIN, DIN2_PIN, 1);

    } else {
        targetPosL = count_l;
        targetPosR = count_r;
        setMotor(0, AIN1_PIN, AIN2_PIN, 0);
        setMotor(0, DIN1_PIN, DIN2_PIN, 1);
        ff_fade_factor = 0; 
    }

    last_mode_ground = current_mode_ground;

    // 串口监视器 (原封不动，包括所有的调试输出)
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > 500) {
        Serial.print("[Pitch]"); Serial.print(pitch);
        Serial.print(" [CH1]"); Serial.print(ch1);
        Serial.print(" [CH2]"); Serial.print(ch2);
        Serial.print(" [Mode]"); Serial.print(current_mode_ground ? "Ground" : "Flight");
        Serial.print(" [F]"); Serial.print(distF);
        Serial.print("mm [R]"); Serial.print(distR);
        Serial.println("mm");
        lastPrintTime = millis();
    }
}
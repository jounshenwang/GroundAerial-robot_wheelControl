// ===================== 串口通信模块 (comm.cpp) =====================
// 通过 UART1 与 Jetson Orin Nano (serial_bridge 节点) 交换二进制帧
//
// 核心设计：
//   ① 接收：每周期查询 Serial1.available()，通过有限状态机逐字节解析
//      帧同步 → 定长接收 → XOR 校验 → 字段提取
//   ② 发送：每 20ms 组 15 字节上行帧，原子写入 UART 发送缓冲区
//   ③ 超时保护：连续 200ms 未收到有效下行帧 → 自动切回手动模式
//
// 注意：不使用 Serial0 (已用于 USB 调试输出)
#include "comm.h"
#include "encoder.h"
#include "imu.h"
#include "safety.h"

namespace Comm {

// ======================== 内部状态 ========================

// 解析结果缓存（供 main.cpp 读取）
static volatile int16_t s_targetVelL = 0;
static volatile int16_t s_targetVelR = 0;
static volatile uint8_t s_mode      = MODE_MANUAL;
static volatile uint32_t s_lastRxMs = 0;

// RX 有限状态机
enum RxState : uint8_t {
    RX_WAIT_SYNC,    // 等待帧头 0xAA
    RX_RECV_DATA,    // 接收数据载荷
    RX_VERIFY,       // 校验并解析
};
static RxState  s_rxState = RX_WAIT_SYNC;
static uint8_t  s_rxBuf[COMM_DOWN_LEN];
static uint8_t  s_rxIdx  = 0;

// TX 节拍控制
static uint32_t s_lastTxMs = 0;

// ======================== 内部辅助 ========================

/// XOR 校验：逐个字节异或，返回 8 位校验和
static uint8_t calcChecksum(const uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

// ======================== 接收状态机 ========================

static void rxProcessByte(uint8_t byte) {
    switch (s_rxState) {

    case RX_WAIT_SYNC:
        if (byte == COMM_SYNC_DOWN) {
            s_rxBuf[0] = byte;
            s_rxIdx    = 1;
            s_rxState  = RX_RECV_DATA;
        }
        // 非同步字节直接丢弃（保持同步搜索状态）
        break;

    case RX_RECV_DATA:
        s_rxBuf[s_rxIdx++] = byte;
        if (s_rxIdx >= COMM_DOWN_LEN) {
            s_rxState = RX_VERIFY;
        }
        break;

    case RX_VERIFY: {
        // 计算前 COMM_DOWN_LEN-1 字节的 XOR 校验
        uint8_t cs = calcChecksum(s_rxBuf, COMM_DOWN_LEN - 1);
        if (cs == s_rxBuf[COMM_DOWN_LEN - 1]) {
            // 校验通过 → 解析下行帧字段
            DownFrame* frame = (DownFrame*)s_rxBuf;
            s_targetVelL = frame->vTargetL;
            s_targetVelR = frame->vTargetR;
            s_mode       = frame->mode;
            s_lastRxMs   = millis();
        }
        // 无论校验成功与否，复位状态机等待下一帧
        s_rxState = RX_WAIT_SYNC;
        s_rxIdx   = 0;
        break;
    }
    }
}

static void commReceive() {
    while (Serial1.available()) {
        uint8_t byte = Serial1.read();
        rxProcessByte(byte);
    }

    // ═══ 通信超时保护 ═══
    // 在 ROS2 模式下，若超过 COMM_RX_TIMEOUT 未收到有效下行帧，
    // 自动切换回手动模式并将速度指令归零（安全降级行为）
    uint32_t now = millis();
    if (s_mode == MODE_ROS2_AUTO && (now - s_lastRxMs > COMM_RX_TIMEOUT)) {
        s_mode = MODE_MANUAL;
        s_targetVelL = 0;
        s_targetVelR = 0;
    }
}

// ======================== 发送 ========================

static void commTransmit() {
    UpFrame frame;
    memset(&frame, 0, sizeof(frame));

    frame.sync = COMM_SYNC_UP;

    // 编码器累计计数（原子读取）
    long encL, encR;
    Encoder::readPair(encL, encR);
    frame.encL = (int32_t)encL;
    frame.encR = (int32_t)encR;

    // 俯仰角 ×100（定点化，避免浮点传输歧义）
    frame.pitch = (int16_t)(IMU::angle_pitch * 100.0f);

    // 故障位掩码 + 安全状态
    frame.fault = Safety::faultFlags;
    frame.state = (uint8_t)Safety::state;

    // 校验和（前 14 字节 XOR）
    frame.checksum = calcChecksum((uint8_t*)&frame, sizeof(frame) - 1);

    Serial1.write((uint8_t*)&frame, sizeof(frame));
}

// ======================== 公共接口 ========================

void commInit() {
    // UART1: 921600, 8N1, RX=GPIO44, TX=GPIO43
    Serial1.begin(COMM_BAUD, SERIAL_8N1, COMM_RX_PIN, COMM_TX_PIN);

    s_rxState  = RX_WAIT_SYNC;
    s_rxIdx    = 0;
    s_mode     = MODE_MANUAL;
    s_targetVelL = 0;
    s_targetVelR = 0;
    s_lastRxMs = 0;
    s_lastTxMs = 0;
}

void commUpdate() {
    // ① 接收下行帧（ROS2 → ESP32），解析速度指令和模式
    commReceive();

    // ② 发送上行帧（ESP32 → ROS2），逐周期上报状态
    // 节拍与控制周期同步（20ms），避免 UART 发送队列堆积
    uint32_t now = millis();
    if (now - s_lastTxMs >= COMM_FRAME_PERIOD) {
        commTransmit();
        s_lastTxMs = now;
    }
}

int16_t getTargetVelocityL() { return s_targetVelL; }
int16_t getTargetVelocityR() { return s_targetVelR; }
uint8_t getMode()            { return s_mode; }
uint32_t getRxAge()          { return millis() - s_lastRxMs; }

} // namespace Comm

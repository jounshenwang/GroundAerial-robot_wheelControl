// ===================== MAVLink 飞控通信模块 (mavlink_bridge.cpp) =====================
//
// MAVLink v2 帧结构 (无签名):
//   [0]=0xFD [1]=len [2]=incompat [3]=compat [4]=seq [5]=sys [6]=comp
//   [7..9]=msgId(3B LE) [10..10+len-1]=payload [..]=CRC16(LE)
//   CRC-16/MCRF4XX 覆盖 buf[1]..buf[10+len-1] + CRC_EXTRA 字节, poly=0x1021
//
// 实现的消息:
//   HEARTBEAT(0)      — 收发心跳
//   COMMAND_LONG(76)  — 发送 arm/disarm / flight termination
//   COMMAND_ACK(77)   — 接收执行结果
// =====================

#include "mavlink_bridge.h"
#include <cstring>    // memcpy, memset

#ifdef MAVLINK_ENABLED

namespace MavlinkBridge {

// =========================== 常量 ===========================

static const uint8_t  STX              = 0xFD;
static const uint8_t  CRC_EXTRA_HB     = 50;   // HEARTBEAT
static const uint8_t  CRC_EXTRA_CMD    = 152;  // COMMAND_LONG
static const uint8_t  CRC_EXTRA_ACK    = 143;  // COMMAND_ACK

static const uint16_t MSG_HEARTBEAT    = 0;
static const uint16_t MSG_COMMAND_LONG = 76;
static const uint16_t MSG_COMMAND_ACK  = 77;

static const uint16_t CMD_ARM_DISARM   = 400;
static const uint16_t CMD_FLIGHT_TERM  = 185;

static const uint8_t  ARM_BIT          = 0x80;  // base_mode bit 7 → armed

// =========================== CRC-16/MCRF4XX 查找表 ===========================
// poly=0x1021, init=0xFFFF, ref_in=true, ref_out=true, xor_out=0x0000

static const uint16_t CRC_TABLE[256] = {
    0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,
    0x8C48,0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7,
    0x1081,0x0108,0x3393,0x221A,0x56A5,0x472C,0x75B7,0x643E,
    0x9CC9,0x8D40,0xBFDB,0xAE52,0xDAED,0xCB64,0xF9FF,0xE876,
    0x2102,0x308B,0x0210,0x1399,0x6726,0x76AF,0x4434,0x55BD,
    0xAD4A,0xBCC3,0x8E58,0x9FD1,0xEB6E,0xFAE7,0xC87C,0xD9F5,
    0x3183,0x200A,0x1291,0x0318,0x77A7,0x662E,0x54B5,0x453C,
    0xBDCB,0xAC42,0x9ED9,0x8F50,0xFBEF,0xEA66,0xD8FD,0xC974,
    0x4204,0x538D,0x6116,0x709F,0x0420,0x15A9,0x2732,0x36BB,
    0xCE4C,0xDFC5,0xED5E,0xFCD7,0x8868,0x99E1,0xAB7A,0xBAF3,
    0x5285,0x430C,0x7197,0x601E,0x14A1,0x0528,0x37B3,0x263A,
    0xDECD,0xCF44,0xFDDF,0xEC56,0x98E9,0x8960,0xBBFB,0xAA72,
    0x6306,0x728F,0x4014,0x519D,0x2522,0x34AB,0x0630,0x17B9,
    0xEF4E,0xFEC7,0xCC5C,0xDDD5,0xA96A,0xB8E3,0x8A78,0x9BF1,
    0x7387,0x620E,0x5095,0x411C,0x35A3,0x242A,0x16B1,0x0738,
    0xFFCF,0xEE46,0xDCDD,0xCD54,0xB9EB,0xA862,0x9AF9,0x8B70,
    0x8408,0x9581,0xA71A,0xB693,0xC22C,0xD3A5,0xE13E,0xF0B7,
    0x0840,0x19C9,0x2B52,0x3ADB,0x4E64,0x5FED,0x6D76,0x7CFF,
    0x9489,0x8500,0xB79B,0xA612,0xD2AD,0xC324,0xF1BF,0xE036,
    0x18C1,0x0948,0x3BD3,0x2A5A,0x5EE5,0x4F6C,0x7DF7,0x6C7E,
    0xA50A,0xB483,0x8618,0x9791,0xE32E,0xF2A7,0xC03C,0xD1B5,
    0x2942,0x38CB,0x0A50,0x1BD9,0x6F66,0x7EEF,0x4C74,0x5DFD,
    0xB58B,0xA402,0x9699,0x8710,0xF3AF,0xE226,0xD0BD,0xC134,
    0x39C3,0x284A,0x1AD1,0x0B58,0x7FE7,0x6E6E,0x5CF5,0x4D7C,
    0xC60C,0xD785,0xE51E,0xF497,0x8028,0x91A1,0xA33A,0xB2B3,
    0x4A44,0x5BCD,0x6956,0x78DF,0x0C60,0x1DE9,0x2F72,0x3EFB,
    0xD68D,0xC704,0xF59F,0xE416,0x90A9,0x8120,0xB3BB,0xA232,
    0x5AC5,0x4B4C,0x79D7,0x685E,0x1CE1,0x0D68,0x3FF3,0x2E7A,
    0xE70E,0xF687,0xC41C,0xD595,0xA12A,0xB0A3,0x8238,0x93B1,
    0x6B46,0x7ACF,0x4854,0x59DD,0x2D62,0x3CEB,0x0E70,0x1FF9,
    0xF78F,0xE606,0xD49D,0xC514,0xB1AB,0xA022,0x92B9,0x8330,
    0x7BC7,0x6A4E,0x58D5,0x495C,0x3DE3,0x2C6A,0x1EF1,0x0F78,
};

static inline uint16_t crcUpdate(uint8_t b, uint16_t crc) {
    return (crc >> 8) ^ CRC_TABLE[(crc ^ b) & 0xFF];
}

// =========================== 内部状态 ===========================

static HardwareSerial* s = nullptr;  // &Serial2
static uint8_t         s_seq;        // 发送序列号

static uint32_t s_lastTxHbMs;        // 上次发送 HEARTBEAT 时间
static uint32_t s_lastFcuHbMs;       // 上次收到飞控 HEARTBEAT 时间

static uint8_t s_fcuSysId   = 1;     // 飞控 sysid (从帧头自动采集)
static uint8_t s_fcuCompId  = 1;
static uint8_t s_fcuBaseMode;        // 飞控 base_mode (bit7=armed)
static bool    s_fcuOnline;

// ── RX 状态机 ──
enum RxS : uint8_t { WAIT, HDR, PAYLOAD, CRC };
static RxS      s_st = WAIT;
static uint8_t  s_buf[280];
static uint16_t s_pos;
static uint8_t  s_plen;   // payload 长度 (buf[1])
static bool     s_sign;   // incompat_flags & 0x01 → 有 13B 签名

// =========================== 发送 ===========================

/// 组装并发送一帧 MAVLink v2 消息 (无签名)
static void txFrame(uint32_t msgId, const uint8_t* payload, uint8_t len,
                    uint8_t crcExtra) {
    if (!s) return;
    uint8_t buf[280];
    uint16_t n = 0;

    buf[n++] = STX;
    buf[n++] = len;
    buf[n++] = 0;                           // incompat
    buf[n++] = 0;                           // compat
    buf[n++] = s_seq++;
    buf[n++] = MAVLINK_SYS_ID;
    buf[n++] = MAVLINK_COMP_ID;
    buf[n++] = msgId & 0xFF;                // msgid LSB
    buf[n++] = (msgId >> 8) & 0xFF;
    buf[n++] = (msgId >> 16) & 0xFF;

    memcpy(&buf[n], payload, len); n += len;

    // CRC-16 覆盖 buf[1]..buf[n-1] + crcExtra
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 1; i < n; i++) crc = crcUpdate(buf[i], crc);
    crc = crcUpdate(crcExtra, crc);
    buf[n++] = crc & 0xFF;
    buf[n++] = (crc >> 8) & 0xFF;

    s->write(buf, n);
}

static void sendHeartbeat() {
    // 19=MAV_TYPE_ONBOARD_CONTROLLER, 8=MAV_AUTOPILOT_INVALID,
    // base_mode=0, custom_mode=0, system_status=3 (MAV_STATE_STANDBY)
    const uint8_t p[9] = {19, 8, 0, 0,0,0,0, 3};
    txFrame(MSG_HEARTBEAT, p, 9, CRC_EXTRA_HB);
}

static void sendCommandLong(uint16_t cmd, float p1, float p2, float p3,
                            float p4, float p5, float p6, float p7) {
    uint8_t p[32] = {};
    p[0] = s_fcuSysId;
    p[1] = s_fcuCompId;
    p[2] = cmd & 0xFF;
    p[3] = (cmd >> 8) & 0xFF;
    // p[4]=confirmation=0
    memcpy(&p[5],  &p1, 4);
    memcpy(&p[9],  &p2, 4);
    memcpy(&p[13], &p3, 4);
    memcpy(&p[17], &p4, 4);
    memcpy(&p[21], &p5, 4);
    memcpy(&p[25], &p6, 4);
    memcpy(&p[29], &p7, 4);
    txFrame(MSG_COMMAND_LONG, p, 32, CRC_EXTRA_CMD);
}

// =========================== 接收 ===========================

static void onHb(const uint8_t* p, uint8_t len) {
    if (len < 9) return;
    s_fcuBaseMode = p[2];
    s_lastFcuHbMs = millis();
    s_fcuOnline   = true;
}

static void onAck(const uint8_t* p, uint8_t len) {
    if (len < 3) return;
    // uint16_t cmd  = p[0] | ((uint16_t)p[1] << 8);
    // uint8_t  result = p[2];
    // 调试时取消: Serial.printf("[MAV] ACK cmd=%u res=%u\n", cmd, result);
    (void)p; (void)len;
}

static void onFrame(const uint8_t* buf) {
    uint8_t  plen    = buf[1];
    uint8_t  sysId   = buf[5];
    uint8_t  compId  = buf[6];
    uint32_t msgId   = buf[7] | ((uint32_t)buf[8] << 8)
                      | ((uint32_t)buf[9] << 16);
    const uint8_t* payload = &buf[10];

    if (sysId  != 0) s_fcuSysId  = sysId;
    if (compId != 0) s_fcuCompId = compId;

    switch (msgId) {
    case MSG_HEARTBEAT:    onHb(payload, plen);     break;
    case MSG_COMMAND_ACK:  onAck(payload, plen);    break;
    }
}

static void rxByte(uint8_t b) {
    switch (s_st) {
    case WAIT:
        if (b == STX) { s_buf[0] = b; s_pos = 1; s_st = HDR; }
        break;
    case HDR:
        s_buf[s_pos++] = b;
        if (s_pos >= 10) {
            s_plen = s_buf[1];
            s_sign = (s_buf[2] & 0x01);
            s_st   = s_plen ? PAYLOAD : CRC;
        }
        break;
    case PAYLOAD:
        s_buf[s_pos++] = b;
        if (s_pos >= (uint16_t)(10 + s_plen)) s_st = CRC;
        break;
    case CRC: {
        s_buf[s_pos++] = b;
        uint8_t crcLen = s_sign ? 15 : 2;
        if (s_pos >= (uint16_t)(10 + s_plen + crcLen)) {
            uint16_t end   = 10 + s_plen;
            uint16_t rxCrc = s_buf[end] | ((uint16_t)s_buf[end + 1] << 8);
            uint16_t crc   = 0xFFFF;
            for (uint16_t i = 1; i < end; i++) crc = crcUpdate(s_buf[i], crc);
            // 查消息对应 crc_extra
            uint32_t mid = s_buf[7] | ((uint32_t)s_buf[8] << 8) | ((uint32_t)s_buf[9] << 16);
            uint8_t x = 0;
            switch (mid) {
            case MSG_HEARTBEAT:    x = CRC_EXTRA_HB;  break;
            case MSG_COMMAND_ACK:  x = CRC_EXTRA_ACK; break;
            }
            if (x) {
                crc = crcUpdate(x, crc);
                if (crc == rxCrc) onFrame(s_buf);
            }
            s_st = WAIT; s_pos = 0;
        }
        break;
    }
    }
}

static void recv() {
    if (!s) return;
    while (s->available()) rxByte(s->read());
}

// =========================== 公共接口 ===========================

void begin() {
    s = &Serial2;
    s->begin(MAVLINK_FC_BAUD, SERIAL_8N1,
             MAVLINK_FC_RX_PIN, MAVLINK_FC_TX_PIN);
    s->setRxBufferSize(512);

    s_seq         = 0;
    s_st          = WAIT;
    s_pos         = 0;
    s_fcuOnline   = false;
    s_lastFcuHbMs = 0;
    s_lastTxHbMs  = 0;

    Serial.println("[MAVLink] UART2 init OK, tx=38 rx=39 baud=57600");
}

void update() {
    recv();

    uint32_t now = millis();

    // HEARTBEAT 定时发送
    if (now - s_lastTxHbMs >= MAVLINK_HEARTBEAT_PERIOD) {
        sendHeartbeat();
        s_lastTxHbMs = now;
    }

    // 飞控心跳超时
    if (s_fcuOnline && (now - s_lastFcuHbMs >= MAVLINK_LINK_TIMEOUT)) {
        s_fcuOnline = false;
        Serial.println("[MAVLink] FC heartbeat timeout");
    }
}

bool sendArmDisarm(bool arm) {
    if (!s_fcuOnline) return false;
    sendCommandLong(CMD_ARM_DISARM, arm ? 1.0f : 0.0f,
                    0, 0, 0, 0, 0, 0);
    return true;
}

bool sendFlightTermination() {
    if (!s_fcuOnline) return false;
    sendCommandLong(CMD_FLIGHT_TERM, 0, 0, 0, 0, 0, 0, 0);
    return true;
}

bool    isFcuConnected() { return s_fcuOnline; }
bool    isFcuArmed()     { return (s_fcuBaseMode & ARM_BIT); }
uint8_t fcuSysId()       { return s_fcuSysId; }

} // namespace MavlinkBridge

// =========================== MAVLINK_ENABLED 未定义 → 空桩 ===========================
#else

namespace MavlinkBridge {
    void begin()                 {}
    void update()                {}
    bool sendArmDisarm(bool)     { return false; }
    bool sendFlightTermination() { return false; }
    bool isFcuConnected()        { return false; }
    bool isFcuArmed()            { return false; }
    uint8_t fcuSysId()           { return 0; }
}

#endif // MAVLINK_ENABLED

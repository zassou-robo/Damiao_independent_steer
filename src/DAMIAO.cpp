
#include "DAMIAO.hpp"

#include <cstring>
#include <cmath>

// CAN共通インスタンス
// 現在使用しているCAN1のピン配置を維持
CAN DM3520::can_(PB_12,PB_13, 1000000);

bool DM3520::bus_initialized_ = false;

DM3520* DM3520::instances_[MAX_MOTORS] = {};


// コンストラクタ
DM3520::DM3520(
    uint8_t id,
    Mode mode,
    Range range
)
    : id_(id),
      master_id_(0),
      mode_(mode),
      range_(range)
{
    if (id_ < MAX_MOTORS) {
        instances_[id_] = this;
    }
}


// CAN初期化
bool DM3520::InitBus()
{
    if (bus_initialized_) {
        return true;
    }

    can_.frequency(1000000);

    bus_initialized_ = true;

    return true;
}


// CAN送信
bool DM3520::WriteCAN(
    uint16_t can_id,
    const uint8_t* data,
    uint8_t length
)
{
    if (!bus_initialized_ || length > 8) {
        return false;
    }

    CANMessage msg(
        can_id,
        reinterpret_cast<const char*>(data),
        length,
        CANData,
        CANStandard
    );

    return can_.write(msg);
}


// モーター有効化
bool DM3520::EnableMotor()
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFC
    };

    return WriteCAN(id_, data, 8);
}


// モーター無効化
bool DM3520::DisableMotor()
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFD
    };

    return WriteCAN(id_, data, 8);
}


// モード設定
bool DM3520::SetMode()
{
    uint8_t data[8] = {};

    data[0] = id_ & 0xFF;
    data[1] = (id_ >> 8) & 0xFF;

    data[2] = 0x55;
    data[3] = 0x0A;

    uint32_t mode = static_cast<uint32_t>(mode_);

    data[4] = mode & 0xFF;
    data[5] = (mode >> 8) & 0xFF;
    data[6] = (mode >> 16) & 0xFF;
    data[7] = (mode >> 24) & 0xFF;

    return WriteCAN(0x7FF, data, 8);
}


// モーター初期化
bool DM3520::Init()
{
    if (!bus_initialized_) {
        return false;
    }

    // モード設定
    if (!SetMode()) {
        return false;
    }

    ThisThread::sleep_for(100ms);

    // モーター有効化
    if (!EnableMotor()) {
        return false;
    }

    ThisThread::sleep_for(100ms);

    return true;
}


// Master ID設定
void DM3520::SetMasterID(uint8_t master_id)
{
    master_id_ = master_id;
}


// float → uint
uint32_t DM3520::FloatToUint(
    float value,
    float min,
    float max,
    uint8_t bits
)
{
    if (max <= min || bits == 0 || bits > 16) {
        return 0;
    }

    if (value < min) value = min;
    if (value > max) value = max;

    const uint32_t max_int = (1UL << bits) - 1;

    return static_cast<uint32_t>(
        (value - min) / (max - min) * max_int
    );
}


// uint → float
float DM3520::UintToFloat(
    uint32_t value,
    float min,
    float max,
    uint8_t bits
)
{
    const uint32_t max_int = (1UL << bits) - 1;

    return static_cast<float>(value) /
        max_int * (max - min) + min;
}


// 速度制御
bool DM3520::SendVelocity(float velocity)
{
    if (mode_ != Mode::VELOCITY) {
        return false;
    }

    uint8_t data[4] = {};

    std::memcpy(data, &velocity, sizeof(float));

    // 参考実装ではDLC = 4
    return WriteCAN(
        0x200 + id_,
        data,
        4
    );
}


// 位置・速度制御
bool DM3520::SendPosition(
    float position,
    float max_velocity
)
{
    if (mode_ != Mode::POSITION_VELOCITY) {
        return false;
    }

    uint8_t data[8] = {};

    std::memcpy(
        &data[0],
        &position,
        sizeof(float)
    );

    std::memcpy(
        &data[4],
        &max_velocity,
        sizeof(float)
    );

    return WriteCAN(
        0x100 + id_,
        data,
        8
    );
}


// MIT制御
bool DM3520::SendMIT(
    float position,
    float velocity,
    float kp,
    float kd,
    float torque
)
{
    if (mode_ != Mode::MIT) {
        return false;
    }

    uint16_t p = FloatToUint(
        position,
        -range_.position_max,
        range_.position_max,
        16
    );

    uint16_t v = FloatToUint(
        velocity,
        -range_.velocity_max,
        range_.velocity_max,
        12
    );

    uint16_t kp_int = FloatToUint(
        kp, 0.0f, 500.0f, 12
    );

    uint16_t kd_int = FloatToUint(
        kd, 0.0f, 5.0f, 12
    );

    uint16_t t = FloatToUint(
        torque,
        -range_.torque_max,
        range_.torque_max,
        12
    );

    uint8_t data[8];

    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;

    data[2] = (v >> 4) & 0xFF;

    data[3] =
        ((v & 0x0F) << 4) |
        ((kp_int >> 8) & 0x0F);

    data[4] = kp_int & 0xFF;

    data[5] = (kd_int >> 4) & 0xFF;

    data[6] =
        ((kd_int & 0x0F) << 4) |
        ((t >> 8) & 0x0F);

    data[7] = t & 0xFF;

    return WriteCAN(id_, data, 8);
}


// トルク指令
bool DM3520::SendTorque(float torque)
{
    if (mode_ != Mode::MIT) {
        return false;
    }

    return SendMIT(
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        torque
    );
}


// フィードバック解析
bool DM3520::ParseFeedback(const CANMessage& msg)
{
    if (msg.len != 8) {
        return false;
    }

    // Master IDが0の場合はByte0内のIDで識別
    if (master_id_ == 0) {
        uint8_t received_id = msg.data[0] & 0x0F;

        if (received_id != id_) {
            return false;
        }
    }
    else {
        if (msg.id != master_id_) {
            return false;
        }
    }

    // エラー状態
    feedback_.error = (msg.data[0] >> 4) & 0x0F;

    // 位置 16bit
    uint16_t pos_int =
        (static_cast<uint16_t>(msg.data[1]) << 8) |
        msg.data[2];

    // 速度 12bit
    uint16_t vel_int =
        (static_cast<uint16_t>(msg.data[3]) << 4) |
        (msg.data[4] >> 4);

    // トルク 12bit
    uint16_t torque_int =
        (static_cast<uint16_t>(msg.data[4] & 0x0F) << 8) |
        msg.data[5];

    feedback_.position = UintToFloat(
        pos_int,
        -range_.position_max,
        range_.position_max,
        16
    );

    feedback_.velocity = UintToFloat(
        vel_int,
        -range_.velocity_max,
        range_.velocity_max,
        12
    );

    feedback_.torque = UintToFloat(
        torque_int,
        -range_.torque_max,
        range_.torque_max,
        12
    );

    feedback_.mos_temperature = msg.data[6];
    feedback_.motor_temperature = msg.data[7];

    feedback_.valid = true;

    return true;
}


// CAN受信処理
// void DM3520::Receive()
// {
//     CANMessage msg;

//     while (can_.read(msg)) {

//         // printf(
//         //     "RX ID: 0x%03X DLC: %d DATA:",
//         //     msg.id,
//         //     msg.len
//         // );

//         // for (int i = 0; i < msg.len; i++) {
//             // printf(" %02X",
//             //     static_cast<uint8_t>(msg.data[i])
//             // );
//         // }

//         // printf("\r\n");

//         for (uint8_t i = 0; i < MAX_MOTORS; i++) {
//             DM3520* motor = instances_[i];

//             if (motor == nullptr) {
//                 continue;
//             }

//             if (motor->ParseFeedback(msg)) {
//                 break;
//             }
//         }
//     }
// }
// CAN受信処理
// 対象モーターのフィードバックを1件以上解析できた場合true
bool DM3520::Receive()
{
    bool feedback_received = false;
    CANMessage msg;

    // 受信キューにあるフレームをすべて処理する
    while (can_.read(msg)) {
        printf(
            "RAW RX: ID=0x%03X DLC=%d\r\n",
            msg.id,
            msg.len
        );

        for (int i = 0; i < msg.len; i++) {
            printf("%02X ", static_cast<uint8_t>(msg.data[i]));
        }
        printf("\r\n");

        for (uint8_t i = 0; i < MAX_MOTORS; i++) {
            DM3520* motor = instances_[i];
            if (motor == nullptr) {
                continue;
            }

            if (motor->ParseFeedback(msg)) {
                feedback_received = true;
                break;
            }
        }
    }

    return feedback_received;
}


// CANコントローラ復旧
// reset後にCAN周波数を再設定する
bool DM3520::RecoverBus()
{
    if (!bus_initialized_) {
        return false;
    }

    printf("[CAN] Before reset: TXERR=%u RXERR=%u\r\n",
           static_cast<unsigned>(can_.tderror()),
           static_cast<unsigned>(can_.rderror()));

    can_.reset();
    can_.frequency(1000000);

    printf("[CAN] After reset: TXERR=%u RXERR=%u\r\n",
           static_cast<unsigned>(can_.tderror()),
           static_cast<unsigned>(can_.rderror()));
    printf("[CAN] Reset and 1Mbps reconfiguration completed\r\n");

    return true;
}


// 旧テスト関数との互換性を維持
void DM3520::ResetBusForTest()
{
    RecoverBus();
}


// フィードバック取得
const DM3520::Feedback& DM3520::GetFeedback() const
{
    return feedback_;
}


// モーターID
uint8_t DM3520::GetID() const
{
    return id_;
}


// Master ID
uint8_t DM3520::GetMasterID() const
{
    return master_id_;
}


// モード
DM3520::Mode DM3520::GetMode() const
{
    return mode_;
}



#ifndef DAMIAO_HPP
#define DAMIAO_HPP

#include "mbed.h"
#include <cstdint>

class DM3520 {
public:

    enum class Mode : uint8_t {
        MIT = 1,
        POSITION_VELOCITY = 2,
        VELOCITY = 3
    };

    struct Range {
        float position_max;
        float velocity_max;
        float torque_max;

        Range(
            float p = 12.5f,
            float v = 30.0f,
            float t = 10.0f
        )
            : position_max(p),
              velocity_max(v),
              torque_max(t)
        {}
    };

    struct Feedback {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;

        uint8_t mos_temperature = 0;
        uint8_t motor_temperature = 0;
        uint8_t error = 0;

        bool valid = false;
    };

    // コンストラクタ
    DM3520(
        uint8_t id,
        Mode mode,
        Range range = Range{}
    );

    // CAN初期化
    static bool InitBus();

    // モーター初期化
    bool Init();

    // モーター有効・無効
    bool EnableMotor();
    bool DisableMotor();

    // 制御モード
    bool SetMode();

    // Master ID設定
    void SetMasterID(uint8_t master_id);

    // 制御指令
    bool SendVelocity(float velocity);

    bool SendPosition(
        float position,
        float max_velocity
    );

    bool SendMIT(
        float position,
        float velocity,
        float kp,
        float kd,
        float torque
    );

    bool SendTorque(float torque);

    // フィードバック
    const Feedback& GetFeedback() const;

    // CAN受信。対象モーターのフィードバックを解析できた場合true
    static bool Receive();

    // CANコントローラをリセットして通信速度を再設定
    static bool RecoverBus();

    // モーター情報
    uint8_t GetID() const;
    uint8_t GetMasterID() const;
    Mode GetMode() const;
    static void ResetBusForTest();

private:

    static constexpr uint8_t MAX_MOTORS = 16;

    static DM3520* instances_[MAX_MOTORS];

    static CAN can_;
    static bool bus_initialized_;

    uint8_t id_;
    uint8_t master_id_;

    Mode mode_;
    Range range_;

    Feedback feedback_;

    bool WriteCAN(
        uint16_t can_id,
        const uint8_t* data,
        uint8_t length
    );

    bool ParseFeedback(const CANMessage& msg);

    static uint32_t FloatToUint(
        float value,
        float min,
        float max,
        uint8_t bits
    );

    static float UintToFloat(
        uint32_t value,
        float min,
        float max,
        uint8_t bits
    );
};

#endif

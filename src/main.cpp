#include "mbed.h"
#include "DAMIAO.hpp"
#include "controler.hpp"
#include "C620.hpp"
#include "PID_new.hpp"

// モーター設定
DM3520::Range motor_range = {
    12.5f,  // Position Max
    30.0f,  // Velocity Max
    10.0f   // Torque Max
};
DM3520 motor_1(//right
    1,
    DM3520::Mode::VELOCITY,
    motor_range
);
DM3520 motor_2(//left
    2,
    DM3520::Mode::VELOCITY,
    motor_range
);
dji::C620 c620(PB_12,PB_13);
CAN arduino(PA_11,PA_12, (int)1e6);
PidGain gain_c610{0.5, 0.1, 0.0};
constexpr int NUM_MOTORS = 2;
constexpr int c610_max = 10000;
constexpr int c610_min = -10000;
int16_t bl_goal_rpm[8] = {0};
int16_t bl_actu_rpm[8] = {0};
BufferedSerial pc(USBTX,USBRX,115200);
CANMessage msg;
Ps5 ps5;
float right_steer_drive = 0.0;
float left_steer_drive = 0.0;
std::array<Pid, NUM_MOTORS> pids = {{
    Pid({gain_c610, c610_min, c610_max}),
    Pid({gain_c610, c610_min, c610_max})
}};

int main()
{
    // CAN初期化
    if (!DM3520::InitBus()) {
        printf("CAN Init: Failed\r\n");
        while (true) {
            ThisThread::sleep_for(1000ms);
        }
    }
    printf("CAN Init: OK\r\n");

    // 初回のモード設定とEnable
    if (!motor_1.Init()) {
        printf("Motor1 Init: Failed\r\n");
        while (true) {
            ThisThread::sleep_for(1000ms);
        }
    }
    if (!motor_2.Init()) {
        printf("Motor2 Init: Failed\r\n");
        while (true) {
            ThisThread::sleep_for(1000ms);
        }
    }
    c620.read_data();

    printf("Motor Init: OK\r\n");
    printf("Automatic recovery: ENABLED\r\n");
    printf("Velocity command: DISABLED\r\n");

    // フィードバックが500ms途絶えたら復旧処理を開始
    Timer feedback_timeout;
    Timer retry_interval;
    feedback_timeout.start();
    retry_interval.start();

    // bool recovery_attempted = false;

    while (true) {
        auto now = HighResClock::now();
        static auto pre = now;
        if(ps5.read(arduino)){
        // 受信できたフレームが対象モーターのフィードバックなら監視時間を更新f (DM3520::Receive()) {
        //     feedback_timeout.reset();
        //     recovery_attempted = false;
        // }
        // if (DM3520::Receive()) {
        //     feedback_timeout.reset();
        //     recovery_attempted = false;
        // }

        // const bool feedback_lost =
        //     feedback_timeout.elapsed_time() >= 500ms;

        // const bool retry_ready =
        //     !recovery_attempted || retry_interval.elapsed_time() >= 1000ms;

        // if (feedback_lost && retry_ready) {
        //     printf("\r\n[RECOVERY] Feedback timeout\r\n");

        //     // CANリセット + 1Mbps再設定
        //     if (!DM3520::RecoverBus()) {
        //         printf("[RECOVERY] CAN recovery failed\r\n");
        //     } else {
        //         ThisThread::sleep_for(10ms);

        //         // Enableのみ送信。速度指令は送らない
        //         // if (motor_1.EnableMotor()) {
        //         //     printf("[RECOVERY] Enable: Send OK\r\n");
        //         // } else {
        //         //     printf("[RECOVERY] Enable: Send Failed\r\n");
        //         // }
        //         // if (motor_2.EnableMotor()) {
        //         //     printf("[RECOVERY] Enable: Send OK\r\n");
        //         // } else {
        //         //     printf("[RECOVERY] Enable: Send Failed\r\n");
        //         // }
        //     }

        //     recovery_attempted = true;
        //     retry_interval.reset();
        // }

        // ThisThread::sleep_for(10ms);
        // printf("%d\n",ps5.lstick_x);
        if(abs(ps5.lstick_y) > 40){
            left_steer_drive = ps5.lstick_y / 127 * 10;
        }
        if(abs(ps5.rstick_y) > 40){
            right_steer_drive = ps5.rstick_y / 127 * 10;
        }
        if(ps5.r1){
            bl_goal_rpm[3] = 2000;
        }else if(ps5.r2 > 40){
            bl_goal_rpm[3] = -2000;
        }else{
            bl_goal_rpm[3] = 0;
        }
        if(ps5.l1){
            bl_goal_rpm[2] = 2000;
        }else if(ps5.l2 > 40){
            bl_goal_rpm[2] = -2000;
        }else{
            bl_goal_rpm[2] = 0;
        }
        // for(int i = 0; i < 8; i ++){
        //     bl_actu_rpm[i] = c620.get_rpm(i + 1);
        // }
        if(now - pre > 10ms){
            for(int i = 0; i < 8; i++){
                c620.read_data();
                bl_actu_rpm[i] = c620.get_rpm(i+1);
                // printf("%d   ",bl_actu_rpm[i]);
            }
            printf("\n");
            motor_1.SendVelocity(right_steer_drive);
            motor_2.SendVelocity(left_steer_drive);
            // printf("%d,%d\n",bl_goal_rpm[2],bl_actu_rpm[2]);
            for(int i = 0; i < 2; i++){
                c620.set_output(static_cast<int16_t>(pids[i].calc(bl_goal_rpm[i + 2],bl_actu_rpm[i + 2],0.01f)),i + 3);
            }
            c620.write();
            pre = now;
        }
    }
    }
}

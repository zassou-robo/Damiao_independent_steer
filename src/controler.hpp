#ifndef CONTROLER_HPP
#define CONTROLER_HPP

#include <mbed.h>

struct Ps5
{
    int8_t lstick_x = 0;
    int8_t lstick_y = 0;
    int8_t rstick_x = 0;
    int8_t rstick_y = 0;
    uint8_t l2 = 0;
    uint8_t r2 = 0;

    bool right = 0;
    bool up = 0;
    bool left = 0;
    bool down = 0;
    bool circle = 0;
    bool triangle = 0;
    bool square = 0;
    bool cross = 0;
    bool l1 = 0;
    bool r1 = 0;
    bool l3 = 0;
    bool r3 = 0;
    bool option = 0;
    bool share = 0;

    void parse(CANMessage msg)
    {
        switch (msg.id)
        {
            case 50:
            lstick_x = msg.data[0];
            lstick_y = msg.data[1];
            rstick_x = msg.data[2];
            rstick_y = msg.data[3];
            l2 = msg.data[4];
            r2 = msg.data[5];
            break;

            case 51:
            // 上位バイトはビットで管理されているためビット抽出で確実に bool にする
            right = (msg.data[0] >> 3) & 1;
            up = (msg.data[0] >> 2) & 1;
            left = (msg.data[0] >> 1) & 1;
            down = (msg.data[0] >> 0) & 1;
            circle = (msg.data[1] >> 3) & 1;
            triangle = (msg.data[1] >> 2) & 1;
            square = (msg.data[1] >> 1) & 1;
            cross = (msg.data[1] >> 0) & 1;

            // 下位バイトはボタン毎に 0/1 が入っている想定なので最低ビットを使う
            l1 = (msg.data[2] & 0x01);
            r1 = (msg.data[3] & 0x01);
            l3 = (msg.data[4] & 0x01);
            r3 = (msg.data[5] & 0x01);
            option = (msg.data[6] & 0x01);
            share = (msg.data[7] & 0x01);
            break;
        }
    }

    bool read(CAN& can)
    {
        CANMessage msg;
        // まず CAN バッファから読み取り、読み取れなければ false を返す
        if (!can.read(msg)) {
            return false;
        }
        // id が 50 or 51 のメッセージのみパースする
        if (msg.id == 50 || msg.id == 51) {
            parse(msg);
            return true;
        }
        return false;
    }
};

#endif // CONTROLER_HPP
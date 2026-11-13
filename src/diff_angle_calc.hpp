#ifndef DIFF_ANGLE_CALC_HPP
#define DIFF_ANGLE_CALC_HPP

#include "mbed.h"
int32_t diff_angle_calc(int16_t current_angle, int16_t previous_angle)
{
    int32_t diff = current_angle - previous_angle;
    if (diff > 4096)
    {
        diff -= 8192;
    }
    else if (diff < -4096)
    {
        diff += 8192;
    }
    return diff;
}
#endif
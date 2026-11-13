#ifndef POS_PID_HPP
#define POS_PID_HPP
//きむうぉ
//きむ魚

float  pos_pid_calc(float kp,float kd,int32_t diff_pos,int32_t rpm){
    float output = kp * diff_pos - kd * rpm;
    float limit = 10000;
    output = std::clamp(output, -limit, limit);
    return output;
}
float  pos_pid_calc_16(float kp,float kd,int16_t diff_pos,int16_t rpm){
    float output = kp * diff_pos - kd * rpm;
    float limit = 10000;
    output = std::clamp(output, -limit, limit);
    return output;
}

#endif 
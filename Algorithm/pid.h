#ifndef PID_H
#define PID_H
#include <stdint.h>
//PID 相关参数
extern int16_t target_speed ;//目标速度

extern int16_t PID_output;//PID输出

#ifdef __cplusplus
extern "C" {
#endif

class PID {
public:
    PID() = default;
    PID(float kp, float ki, float kd);

    void reset();
    float Position(float target, float measured, float dt,float output_limit, float integral_limit);
    float Delta(float target, float measured, float dt,float output_limit, float integral_limit);
private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float output_limit_ = 0.0f;
    float integral_limit_ = 0.0f;

    float integral_error_ = 0.0f;
    float integral_calculate_ = 0.0f;
    float prev_error_ = 0.0f;
    bool has_prev_error_ = false;
};

extern PID pid;

#ifdef __cplusplus
}
#endif

#endif
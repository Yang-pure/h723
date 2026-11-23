#include "pid.h"
#include "clamp_symmetric.h"

int16_t target_speed = 0;//目标速度
int16_t PID_output = 0;
PID pid{0.0f, 0.0f, 0.0f};  // 全局 PID 对象

PID::PID(float kp, float ki, float kd)
    : kp_{kp}, ki_{ki}, kd_{kd}
{
}

void PID::reset()
{
    integral_error_ = 0.0f;
    integral_calculate_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_error_ = false;
}

float PID::Position(float target, float measured, float dt,float output_limit, float integral_limit)
{
    if (!(dt > 0.0f))
        return 0.0f;
  
    integral_limit_ = integral_limit;
    output_limit_ = output_limit;
    float error = target - measured;
    float proportional = kp_ * error;

    integral_error_ += error * dt;
    integral_calculate_ = ki_ * integral_error_;
    integral_calculate_ = clamp_symmetric(integral_calculate_, integral_limit_);

    float derivative = 0.0f;
    if (has_prev_error_)
        derivative = kd_ * (error - prev_error_) / dt;

    prev_error_ = error;
    has_prev_error_ = true;

    return clamp_symmetric(
        proportional + integral_calculate_ + derivative, output_limit_
    );
}

float PID::Delta(float target, float measured, float dt,float output_limit, float integral_limit)
{

}
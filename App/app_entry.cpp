#include "app_entry.h"
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "bsp_fdcan.h"
#include "Bsp_DJI_Motor.h"
#include "fdcan.h"

/*
 * 本文件按 C++ 编译。上面的头文件已经声明了 C 链接的接口，
 * 下面对同名函数和变量的定义沿用对应的声明。
 */
volatile uint32_t count_ms = 0U;

/* 只供本文件使用，记录上一次计数时的 HAL tick。 */
static uint32_t last_ms = 0U;

static DJI_Motor can1_motor[]=
{
    DJI_Motor(DJI_Motor::Motor_type::GM6020,DJI_Motor::FDCAN_ID::ID1,DJI_Motor::Motor_mode::POS,PID{10.0f,0.1f,0.0f},PID{10.0f,0.1f,0.0f}),
    DJI_Motor(DJI_Motor::Motor_type::GM6020,DJI_Motor::FDCAN_ID::ID2,DJI_Motor::Motor_mode::POS,PID{10.0f,0.1f,0.0f},PID{10.0f,0.1f,0.0f})
};

void APP_OnFDCAN_RX(const FDCAN_RxHeaderTypeDef &RxHeader,const uint8_t *data)
{
    for(auto& motor: can1_motor)
    {
        motor.Update_Data(RxHeader,data);
    }
}

void App_Init(void)
{
    /* 从调用 App_Init 的时刻开 始重新计数。 */
    count_ms = 0U;
    last_ms = HAL_GetTick();
}




void App_Poll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_ms) >= 1U)
    {
        last_ms = now_ms;
        count_ms = count_ms + 1U;
        for (auto& frame : frames)
    {
        for (auto& byte : frame.data)
        byte = 0U;
    }

    for (auto& motor : can1_motor)
    {
        if (!motor.DJI_Encode(frames))
            return;
    }

    for (auto& frame : frames)
    {
        FDCAN_Send_Data(&hfdcan1, frame.id, frame.data, 8U);
    }
    }
    
}
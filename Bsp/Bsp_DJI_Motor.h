#ifndef BSP_DJI_MOTOR_H
#define BSP_DJI_MOTOR_H

#include "../Algorithm/pid.h"
#include "stm32h7xx_hal.h"
#include "string.h"


#ifdef __cplusplus
extern "C"{
#endif
/*
电机类需要什么东西在里面？
类中需要包含电机对象和电机的动作
对象：
    什么电机：GM6020，M3508，M2006，DM4310,DM4340...-->不同的电机还需要不同的编码解码函数
    FDCAN ID
    控制电机的速度，位置，电流/电压
    还需要反馈电机的实时状态：
        电机的转速
        电机的电流
        电机的温度
        电机的位置
        以及 电机是否在线：在电机掉线后DJI电机会重复发送与掉线前一帧相同的can帧，可通过固定次数内fdcan反馈帧的变化来判断电机是否在线
动作：
    对电机进行初始化
    对电机进行控制 
        1. 速度控制
        2. 电流控制     ----->>由此可以看出，电流控制是速度控制的基础，速度控制是位置控制的基础 -->即需要在不同模式下需要采用不同环数的配置
        3. 位置控制
    --> 具体控制应放在Control中
    判断是否为当前电机的反馈帧，把对应反馈存入电机对象中
*/
typedef struct TxFrame
{
    uint32_t id;
    uint8_t data[8];
}TxFrame;

extern TxFrame frames[3];
                         
class DJI_Motor{
public:
    enum class Motor_type {M2006,M3508,GM6020,DM4310,DM4340};
    enum class FDCAN_ID {ID1 = 1,ID2,ID3,ID4,ID5,ID6,ID7,ID8};
    enum class Motor_mode{SPD,POS};
    //速度
    float set_speed{};//Rpm
    //电流
    int16_t command;//为映射值，而非实际值 根据对象而定是电压还是电流
    
    float set_angle{};
    struct Dji_motor_feedback
    {
        int16_t cur_speed;
        uint16_t cur_current;
        //温度
        uint8_t temperature{};//摄氏度
        //位置
        int16_t cur_angle{};
    }feedback;
 
    PID pid[2];//电流环，速度环，位置环

    //根据类型和ID选择对应的控制帧/反馈帧位置
    //速度单环控制
    DJI_Motor(Motor_type type,FDCAN_ID id,Motor_mode mode,PID spd_pid);
    //位置速度双环控制
    DJI_Motor(Motor_type type,FDCAN_ID id,Motor_mode mode,PID spd_pid,PID pos_pid);

    //更新反馈帧
    //需要确定ID类型，DJI电机看反馈帧ID，C620/C610是0x200+ID，6020是0x204+ID(由此可知若id选取不当，6020会与2006/3508其中一位产生冲突)
    //校验反馈帧：CAN ID, DLC, 帧类型
    bool DJI_Decode(const uint8_t *data,struct Dji_motor_feedback *feedback);
    void Update_Data(const FDCAN_RxHeaderTypeDef &RxHeader,const uint8_t *data);
    //驱动函数：需要对data进行编码后，通过发送通用报文函数发送控制帧
    //encode函数需要先判断为6020还是3508/2006
    //必须对同一条can线上的所有电机进行同时编码，然后再进行发送
    //但是每个电机应当是单独编码的
    //编码的数组不能放在电机类中
    uint32_t DJI_Encode(TxFrame (&frames)[3]);
private:
    Motor_type Type;
    FDCAN_ID ID;
    Motor_mode Mode;
};

#ifdef __cplusplus    
}
#endif


#endif 
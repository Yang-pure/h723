#include "Bsp_DJI_Motor.h"
#include "bsp_fdcan.h"
#include "pid.h"
#include "stm32h7xx_hal.h"
#include "string.h"

DJI_Motor::DJI_Motor(Motor_type type,FDCAN_ID id,Motor_mode mode,PID spd_pid)
    :pid{spd_pid}, Type(type), ID(id), Mode(mode)
    {

    }
DJI_Motor::DJI_Motor(Motor_type type,FDCAN_ID id,Motor_mode mode,PID spd_pid,PID pos_pid)
    : pid{spd_pid, pos_pid}, Type(type), ID(id), Mode(mode)
    {

    }

    
bool DJI_Motor::DJI_Decode(const uint8_t *data,struct Dji_motor_feedback *feedback)
{
  feedback->cur_angle = (data[0] << 8) | data[1];
  feedback->cur_speed = (data[2] << 8) | data[3];
  feedback->cur_current = (data[4] << 8) | data[5];
  feedback->temperature = data[6];
  return true;
}


void DJI_Motor::Update_Data(const FDCAN_RxHeaderTypeDef &RxHeader,const uint8_t *RxData)
{
    const uint32_t device_id = static_cast<uint32_t>(ID);
    uint32_t feedback_id{};
    switch(Type)
    {
        case Motor_type::GM6020:
            feedback_id = 0x204 + device_id;
            break;
        case Motor_type::M2006:
        case Motor_type::M3508:
            feedback_id = 0x200 + device_id;
            break;
        default:
            return;
    }
    if (RxHeader.Identifier == feedback_id &&
        RxHeader.IdType == FDCAN_STANDARD_ID &&
        RxHeader.RxFrameType == FDCAN_DATA_FRAME &&
        RxHeader.FDFormat == FDCAN_CLASSIC_CAN &&
        RxHeader.DataLength == FDCAN_DLC_BYTES_8)
    {
      DJI_Decode(RxData, &feedback);
    }

}

uint32_t DJI_Motor::DJI_Encode(TxFrame (&frames)[3])
{
    const uint32_t device_id = static_cast<uint32_t>(ID);
    const uint32_t first_id = device_id <= 4U ? 1U : 5U;

    uint32_t frame_id {};
    switch(Type)
    {
        case Motor_type::GM6020:
        //GM6020 电压控制
            if (device_id < 1U || device_id > 7U)
                return false;
            frame_id = first_id == 1U ? 0x1FFU : 0x2FFU;
            break;

        case Motor_type::M2006:
        case Motor_type::M3508:
            if (device_id < 1U || device_id > 8U)
                return false;
            frame_id = first_id == 1U ? 0x200U : 0x1FFU;
            break;
        default:
            return false;
    }

    const uint32_t offset = (device_id - first_id) * 2;
    uint16_t txcommand ;
    if(Mode == Motor_mode::SPD)
    {
        txcommand = static_cast<uint16_t>(pid->Position(set_speed,feedback.cur_speed,0.001,10000.0f,10000.0f));
    }
    else if(Mode == Motor_mode::POS)
    {
        set_speed = pid[1].Position(set_angle,feedback.cur_angle,0.001,2000.0f,1000.0f);
       const float output = pid[0].Position(set_speed, feedback.cur_speed, 0.001f, 5000.0f, 5000.0f);
        const int16_t signed_command = static_cast<int16_t>(output);
        txcommand = static_cast<uint16_t>(signed_command);
    }
    for(int i=0;i<3;i++)
    {
        if(frame_id == frames[i].id)
        {
            frames[i].data[offset] = static_cast<uint8_t>(txcommand >> 8U);
            frames[i].data[offset+1] = static_cast<uint8_t>(txcommand & 0xFF);
        }
    }
    return true;
}

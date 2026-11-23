#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"




#ifdef __cplusplus
extern "C" {
#endif
struct fdcan_motor_feedback{
  uint16_t angle;
  volatile int16_t speed;  /* 接收中断写入，App 主循环读取 */
  int16_t current;
  uint8_t temperature;
};
extern struct fdcan_motor_feedback GM6020_feedback[4];

//FDCAN初始化函数
void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);

//发送报文函数
uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan,uint32_t id,uint8_t *data,uint8_t len);

//FDCAN滤波函数
void FDCAN_FILTER_MASK_Config(FDCAN_HandleTypeDef *hfdcan,uint32_t id,uint32_t mask_id);

//接收报文中断回调函数
extern volatile uint32_t fdcan_rx_count ;      /* 收到多少帧 */
extern volatile uint32_t fdcan_rx_id ;      /* 最近一帧的 ID */
extern volatile uint32_t fdcan_rx_dlc;      /* 最近一帧的 DLC */
extern volatile uint8_t  fdcan_rx_data[8];        /* 最近一帧的数据 */
void BSP_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);


#ifdef __cplusplus
}
#endif

#endif

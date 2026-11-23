#ifndef APP_ENTRY_H
#define APP_ENTRY_H

#include <stdint.h>
#include "bsp_fdcan.h"

/*
 * main.c 和 app_entry.cpp 共用这份声明。
 * C++ 编译器定义 __cplusplus，下面的接口获得 C 语言链接。
 * C 编译器跳过 extern "C" 的开头和结尾，直接读取内部声明。
 */
#ifdef __cplusplus
extern "C" {
#endif




void App_Init(void);
void App_Poll(void);

/* 这里只声明变量。存储空间和初始值由 app_entry.cpp 提供。 */
extern volatile uint32_t count_ms;

#ifdef __cplusplus
}

void APP_OnFDCAN_RX(const FDCAN_RxHeaderTypeDef &RxHeader,const uint8_t *data);

#endif

#endif
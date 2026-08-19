#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 



void FDCAN1_StartAndConfig(void); // FDCAN1启动并配置CAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs); // FDCAN Rx FIFO 0 回调函数
#endif

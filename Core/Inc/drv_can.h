#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 
#define     CAN_MASTER_ID      0x00A   // 主机ID
#define     CAN_SLAVE_ID       0x00B   // 从机ID


void FDCAN1_StartAndConfig(void); // FDCAN1启动并配置CAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs); // FDCAN Rx FIFO 0 回调函数
HAL_StatusTypeDef FDCAN1_Send(uint32_t id, const uint8_t *data, uint8_t len); // FDCAN1发送函数

typedef void (*DRV_CAN_RxCallback)(uint32_t ID, uint8_t* buf, uint8_t len);

/**
 * @brief CAN接收数据处理
 *
 * 在主循环中调用，负责从CAN接收队列中取出数据，
 * 然后调用已经注册的回调函数。
 */
void DRV_CAN_ProcessData(void);

/**
 * @brief 注册CAN接收回调函数
 */
void DRV_RegisterCanRxCallback(DRV_CAN_RxCallback callback);


#endif

#include <drv_can.h>
#include "fdcan.h"

//////////////////////////////////////////////////////////////////////////////////	 

#define     DRV_CAN_MAX_STD_ID      0x8FF   // 标准帧ID上限
#define     DRV_CAN_MAX_FRAMES      20      // 数据帧缓存上限
#define     DRV_CAN_MAX_CALLBACKS   20      // 函数指针上限


/**
  * @brief  配置 FDCAN 过滤器、启动外设并使能中断
  * @note   请在 main.c 的 MX_FDCAN1_Init() 之后调用此函数
  */
void FDCAN1_StartAndConfig(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* 1. 配置过滤器
     - 接收所有标准帧 (Standard ID)
     - 存入 RX FIFO 0
  */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000; // ID 匹配值
  sFilterConfig.FilterID2 = 0x000; // Mask: 0 表示接收所有 ID

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2. 启动 FDCAN 外设 */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 3. 使能 RX FIFO 0 新消息中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  FDCAN Rx FIFO 0 回调函数
  * @param  hfdcan: FDCAN handle
  * @param  RxFifo0ITs: Interrupt flags
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    /* 获取接收到的消息 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      /* --- 用户处理逻辑开始 --- */

      // 示例：打印或处理数据
      // RxHeader.Identifier : 消息 ID
      // RxData[]            : 数据内容
      // RxHeader.DataLength : 数据长度

      /* --- 用户处理逻辑结束 --- */
    }
  }
}

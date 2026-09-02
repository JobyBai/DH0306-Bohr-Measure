#include <drv_can.h>
#include "fdcan.h"
#include <string.h> 

/* -------------------------------------------------------------------------- */
/* 配置参数                                                                   */
/* -------------------------------------------------------------------------- */
#define     DRV_CAN_MAX_STD_ID      0x7FF
#define     DRV_CAN_MAX_EXT_ID      0x1FFFFFFF
#define     DRV_CAN_MAX_FRAMES      20
#define     DRV_CAN_MAX_CALLBACKS   20

/* -------------------------------------------------------------------------- */
/* CAN数据帧                                                                  */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint32_t ID;
    uint8_t buf[8];
    uint8_t len;
    uint8_t isExt;
} CAN_Frame;

/* -------------------------------------------------------------------------- */
/* CAN接收队列                                                                */
/* -------------------------------------------------------------------------- */
typedef struct {
    CAN_Frame data[DRV_CAN_MAX_FRAMES];
    uint8_t head;
    uint8_t tail;
} CAN_FrameQueue;

/* -------------------------------------------------------------------------- */
/* 静态函数声明                                                               */
/* -------------------------------------------------------------------------- */
static void CAN_QueuePush(CAN_FrameQueue* queue, CAN_Frame frame);
static uint8_t CAN_QueuePop(CAN_FrameQueue* queue, CAN_Frame* frame);
static uint8_t CAN_QueueGetNum(CAN_FrameQueue* queue);

/* -------------------------------------------------------------------------- */
/* 模块内部状态                                                               */
/* -------------------------------------------------------------------------- */
static CAN_FrameQueue canQueue = {0};
static DRV_CAN_RxCallback canRxCallback_table[DRV_CAN_MAX_CALLBACKS]; 
static uint8_t canRxCallback_count = 0; 

// 【新增】全局锁或标志，防止重入问题（可选，视具体RTOS环境而定）
static volatile uint8_t is_sending = 0;

/**
  * @brief  配置 FDCAN 过滤器、启动外设并使能中断
  */
void FDCAN1_StartAndConfig(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* 1. 配置过滤器：接收所有标准帧 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000; // Mask 0 表示关心所有位，即接收所有 ID

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2. 配置 RX FIFO 0 的水位线（可选，防止频繁中断） */
  // HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_OVERWRITE_ENABLE);

  /* 3. 启动 FDCAN 外设 */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 4. 使能 RX FIFO 0 新消息中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  FDCAN1 标准帧发送函数 (增强版)
  * @note   增加了 TxHeader 的显式初始化，防止栈垃圾数据导致 ID 错误
  */
HAL_StatusTypeDef FDCAN1_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader  = {0};

  // 【关键修复】显式清零 Header，防止未初始化字段导致随机 ID 或 RTR 位错误
  // memset(&TxHeader, 0, sizeof(FDCAN_TxHeaderTypeDef));

  /* 参数校验 */
  if (id > DRV_CAN_MAX_STD_ID || len > 8 || data == NULL)
  {
    return HAL_ERROR;
  }

  /* DLC 映射 */
  static const uint32_t dlc_map[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
  };

  TxHeader.Identifier       = id & 0x7FF; // 强制掩码，确保是标准帧
  TxHeader.IdType           = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType      = FDCAN_DATA_FRAME; // 明确指定为数据帧，非远程帧
  TxHeader.DataLength       = dlc_map[len];
  TxHeader.BitRateSwitch    = FDCAN_BRS_OFF;
  TxHeader.FDFormat         = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//  TxHeader.ErrorStateIndicator = FDCAN_ERROR_STATE_OK;
  TxHeader.MessageMarker    = 0;

  // 【调试建议】如果仍然出现奇怪 ID，可以在这里加断点检查 TxHeader.Identifier

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t *)data);
}

/**
  * @brief  CAN数据处理函数
  */
void DRV_CAN_ProcessData(void)
{
    CAN_Frame frame;
    while (CAN_QueueGetNum(&canQueue)) 
    {
        if (CAN_QueuePop(&canQueue, &frame))
        {
            for (uint8_t i = 0; i < canRxCallback_count; i++) 
            {
                if (canRxCallback_table[i] != NULL) 
                {
                    // 回调中不应执行耗时操作
                    canRxCallback_table[i](frame.ID, frame.buf, frame.len);
                }
            }
        }
    }
}

void DRV_RegisterCanRxCallback(DRV_CAN_RxCallback callback)
{
    if (callback == NULL || canRxCallback_count >= DRV_CAN_MAX_CALLBACKS) return;
    canRxCallback_table[canRxCallback_count++] = callback;
}

/**
  * @brief  FDCAN RX FIFO 0 中断回调
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != hfdcan1.Instance) return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];
        CAN_Frame frame;

        // 获取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        {
            return;
        }

        frame.isExt = (rxHeader.IdType == FDCAN_EXTENDED_ID) ? 1 : 0;
        frame.ID = rxHeader.Identifier;

        // 安全获取长度
        switch(rxHeader.DataLength)
        {
            case FDCAN_DLC_BYTES_0: frame.len = 0; break;
            case FDCAN_DLC_BYTES_1: frame.len = 1; break;
            case FDCAN_DLC_BYTES_2: frame.len = 2; break;
            case FDCAN_DLC_BYTES_3: frame.len = 3; break;
            case FDCAN_DLC_BYTES_4: frame.len = 4; break;
            case FDCAN_DLC_BYTES_5: frame.len = 5; break;
            case FDCAN_DLC_BYTES_6: frame.len = 6; break;
            case FDCAN_DLC_BYTES_7: frame.len = 7; break;
            case FDCAN_DLC_BYTES_8: frame.len = 8; break;
            default: frame.len = 8; break; 
        }

        memcpy(frame.buf, rxData, frame.len);
        CAN_QueuePush(&canQueue, frame);
    }
}

// --- 队列操作实现 (保持不变) ---
static void CAN_QueuePush(CAN_FrameQueue* queue, CAN_Frame frame)
{
    if (queue == NULL) return;
    uint8_t next_tail = (queue->tail + 1) % DRV_CAN_MAX_FRAMES;
    if (next_tail != queue->head) 
    {
        queue->data[queue->tail] = frame; 
        queue->tail = next_tail; 
    }
}

static uint8_t CAN_QueuePop(CAN_FrameQueue* queue, CAN_Frame* frame)
{
    if (queue == NULL || frame == NULL) return 0;
    if (queue->head != queue->tail)
    {
        *frame = queue->data[queue->head]; 
        queue->head = (queue->head + 1) % DRV_CAN_MAX_FRAMES; 
        return 1;
    }
    return 0;
}

static uint8_t CAN_QueueGetNum(CAN_FrameQueue* queue)
{
    if (queue == NULL) return 0;
    return (DRV_CAN_MAX_FRAMES + queue->tail - queue->head) % DRV_CAN_MAX_FRAMES;
}

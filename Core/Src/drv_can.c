#include <drv_can.h>
#include "fdcan.h"
#include <string.h> 


/* -------------------------------------------------------------------------- */
/* 配置参数                                                                   */
/* -------------------------------------------------------------------------- */

#define     DRV_CAN_MAX_STD_ID      0x7FF   // 标准帧ID上限 (11位)
#define     DRV_CAN_MAX_EXT_ID      0x1FFFFFFF // 扩展帧ID上限 (29位)
#define     DRV_CAN_MAX_FRAMES      20      // 数据帧缓存上限
#define     DRV_CAN_MAX_CALLBACKS   20      // 函数指针上限


/* -------------------------------------------------------------------------- */
/* CAN数据帧                                                                  */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint32_t ID;
    uint8_t buf[8]; // G4 FDCAN 在 Classic CAN 模式下最大仍为 8 字节，若启用 BRS/FD 模式可更大，此处暂定为8以兼容原有逻辑
    uint8_t len;
    uint8_t isExt;  // 标记是否为扩展帧，可选
} CAN_Frame;


/* -------------------------------------------------------------------------- */
/* CAN接收队列                                                                */
/* -------------------------------------------------------------------------- */

typedef struct {
    CAN_Frame data[DRV_CAN_MAX_FRAMES]; // 队列数据
    uint8_t head; // 队首
    uint8_t tail; // 队尾
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

static CAN_FrameQueue canQueue = {{0}, 0, 0}; 
static DRV_CAN_RxCallback canRxCallback_table[DRV_CAN_MAX_CALLBACKS]; 
static uint8_t canRxCallback_count = 0; 



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
  * @brief  FDCAN1 标准帧发送函数
  * @param  id:      CAN 消息 ID (标准帧范围: 0x000 ~ 0x7FF)
  * @param  data:    待发送数据指针
  * @param  len:     数据长度 (字节数, 有效值: 0~8)
  * @retval HAL_StatusTypeDef: HAL_OK 成功, 其他表示失败
  */
HAL_StatusTypeDef FDCAN1_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;

  /* 参数校验 */
  if (id > 0x7FF || len > 8 || data == NULL)
  {
    return HAL_ERROR;
  }

  /* 根据实际字节长度映射到 FDCAN DLC 枚举值 */
  static const uint32_t dlc_map[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
  };

  TxHeader.Identifier       = id;
  TxHeader.IdType           = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType      = FDCAN_DATA_FRAME;
  TxHeader.DataLength       = dlc_map[len];
  TxHeader.BitRateSwitch    = FDCAN_BRS_OFF;       // 经典 CAN 模式，不开启 BRS
  TxHeader.FDFormat         = FDCAN_CLASSIC_CAN;   // 经典 CAN 帧格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker    = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t *)data);
}


/**
  * @brief  CAN数据处理函数，建议在主循环 while(1) 中调用
  */
void DRV_CAN_ProcessData(void)
{
    CAN_Frame frame;
    // 只要队列不为空，就取出数据并执行回调
    while (CAN_QueueGetNum(&canQueue)) 
    {
        if (CAN_QueuePop(&canQueue, &frame))
        {
            // 遍历所有注册的回调函数
            for (uint8_t i = 0; i < canRxCallback_count; i++) 
            {
                if (canRxCallback_table[i] != NULL) 
                {
                    canRxCallback_table[i](frame.ID, frame.buf, frame.len);
                }
            }
        }
    }
}


/**
  * @brief  注册CAN接收回调函数
  * @param  callback: 回调函数指针
  */
void DRV_RegisterCanRxCallback(DRV_CAN_RxCallback callback)
{
    if (callback == NULL) return;

    if (canRxCallback_count < DRV_CAN_MAX_CALLBACKS) 
    {
        canRxCallback_table[canRxCallback_count] = callback;
        canRxCallback_count++;
    }
}


/**
  * @brief  FDCAN RX FIFO 0 中断回调函数
  * @note   此函数由 HAL 库在中断中自动调用
  * @param  hfdcan: FDCAN handle
  * @param  RxFifo0ITs: 指示哪个中断发生 (这里我们只关心消息挂起)
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 检查是否是预期的句柄
    if (hfdcan->Instance != hfdcan1.Instance) return;

    // 检查是否是 RX FIFO 0 新消息中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8]; // 临时缓冲区
        CAN_Frame frame;

        // 获取接收到的消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        {
            return; // 读取失败
        }

        // 填充帧结构
        frame.isExt = (rxHeader.IdType == FDCAN_EXTENDED_ID) ? 1 : 0;
        
        if (rxHeader.IdType == FDCAN_STANDARD_ID)
        {
            frame.ID = rxHeader.Identifier;
        }
        else // FDCAN_EXTENDED_ID
        {
            frame.ID = rxHeader.Identifier;
        }

        // 获取数据长度 (DLC)
        // 注意：HAL_FDCAN_GetRxMessage 返回的 rxHeader.DataLength 是枚举值，需要转换或直接使用
        // 在 Classic CAN 模式下，DLC 直接对应字节数。
        // FDCAN HAL 库中，DataLength 字段可能是 FDCAN_DLC_BYTES_8 等枚举，或者直接是字节数，视 HAL 版本而定。
        // 通常 HAL 库会提供一个宏或直接映射。这里假设 rxHeader.DataLength 已经是以字节为单位或者我们需要转换。
        // 在较新的 STM32G4 HAL 中，rxHeader.DataLength 往往是 FDCAN_DLC_BYTES_x 枚举。
        
        // 安全获取长度方法：
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
            // 如果启用了 FD 模式，可能还有更多，但这里 buf 只有 8 字节，故截断或忽略
            default: frame.len = 8; break; 
        }

        // 复制数据
        memcpy(frame.buf, rxData, frame.len);

        // 推入队列
        CAN_QueuePush(&canQueue, frame);
    }
}


// --- 队列操作实现 ---

/**
  * @brief  将CAN帧放入队列
  * @param  queue: CAN帧队列指针
  * @param  frame: 要推入的CAN帧
  */
static void CAN_QueuePush(CAN_FrameQueue* queue, CAN_Frame frame)
{
    if (queue == NULL) return;

    uint8_t next_tail = (queue->tail + 1) % DRV_CAN_MAX_FRAMES;
    
    // 判断队列是否满 (如果 next_tail 等于 head，则满)
    if (next_tail != queue->head) 
    {
        queue->data[queue->tail] = frame; 
        queue->tail = next_tail; 
    }
    // 如果满了，可以选择丢弃最旧的数据或当前数据，这里选择直接丢弃当前数据(不覆盖)
}

/**
  * @brief  从队列取出CAN帧
  * @param  queue: CAN帧队列指针
  * @param  frame: 要弹出的CAN帧指针
  * @return uint8_t: 1 成功弹出，0 失败
  */
static uint8_t CAN_QueuePop(CAN_FrameQueue* queue, CAN_Frame* frame)
{
    if (queue == NULL || frame == NULL) return 0;

    if (queue->head != queue->tail) // 非空
    {
        *frame = queue->data[queue->head]; 
        queue->head = (queue->head + 1) % DRV_CAN_MAX_FRAMES; 
        return 1; // 成功弹出
    }
    return 0; // 队列为空
}

/**
  * @brief  获取队列中已存储的CAN帧数量
  * @param  queue: CAN帧队列指针
  * @return uint8_t: 队列中已存储的CAN帧数量
  */
static uint8_t CAN_QueueGetNum(CAN_FrameQueue* queue)
{
    if (queue == NULL) return 0;
    return (DRV_CAN_MAX_FRAMES + queue->tail - queue->head) % DRV_CAN_MAX_FRAMES;
}
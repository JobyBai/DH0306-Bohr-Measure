#include "control.h"
#include "drv_can.h"
#include "main.h"
#include <string.h>
#include "adc.h"
#include "dma.h"
#include "tim.h"


//////////////////////////////////////////////////////////////////////////////////	 
uint8_t CAN_SLAVE_ANSWER[8] = { 0x00, 0x20 };         // 从机应答指令
uint8_t CAN_SLAVE_AMPLITUDE[8] = { 0x00, 0x21 };  // 振幅返回
uint8_t CAN_SLAVE_PERIOD[8] = { 0x00, 0x22 };  // 周期返回
uint8_t CAN_SLAVE_PHASE_DIFF_1[8] = { 0x00, 0x23 };  // 相位差1返回
uint8_t CAN_SLAVE_PHASE_DIFF_2[8] = { 0x00, 0x24 };  // 相位差2返回
uint8_t CAN_SLAVE_ADC_STREAM[8] = { 0x00, 0x25 };// 获取角度ADC值和编码器值


uint16_t s_adc_stream_seq      = 0;   /* 帧序号，接收端用于检测丢帧 */
uint32_t s_adc_stream_last_pos = 0;   /* 上次已转发到的 DMA 索引位置 */


void Control_Init(void) {
	DRV_RegisterCanRxCallback(Control_CanRxCallback); // 注册CAN接收处理回调函数
}

// CAN接收处理回调函数
void Control_CanRxCallback(uint32_t ID, uint8_t *buf, uint8_t len) {
	if (ID != CAN_MASTER_ID)
		return;
	uint8_t function = buf[1];  // 功能码

	// uint8_t can_msg[8];
	switch (function) {
	case 0x20: // 从机应答
		FDCAN1_Send(CAN_SLAVE_ID, CAN_SLAVE_ANSWER, 8);
		break;
	case 0x21: // 获取振幅
		if (analyzer_result.is_valid) {
			memcpy(&CAN_SLAVE_AMPLITUDE[2], &analyzer_result.amplitude_v,
					sizeof(float));
			FDCAN1_Send(CAN_SLAVE_ID, CAN_SLAVE_AMPLITUDE, 8);
		}
		break;
	case 0x22: // 获取周期
		if (analyzer_result.is_valid) {
			memcpy(&CAN_SLAVE_PERIOD[2], &analyzer_result.period_sec,
					sizeof(float));
			FDCAN1_Send(CAN_SLAVE_ID, CAN_SLAVE_PERIOD, 8);
		}
		break;
	case 0x23: // 获取相位差1
		if (analyzer_result.is_valid) {
			memcpy(&CAN_SLAVE_PHASE_DIFF_1[2], &analyzer_result.phase_diff_1,
					sizeof(float));
			FDCAN1_Send(CAN_SLAVE_ID, CAN_SLAVE_PHASE_DIFF_1, 8);
		}
		break;
	case 0x24: // 获取相位差2
		if (analyzer_result.is_valid) {
			memcpy(&CAN_SLAVE_PHASE_DIFF_2[2], &analyzer_result.phase_diff_2,
					sizeof(float));
			FDCAN1_Send(CAN_SLAVE_ID, CAN_SLAVE_PHASE_DIFF_2, 8);
		}
		break;
	default:
		break;
	}
}



/**
 * @brief 实时转发 ADC 采样值到 CAN 总线 (每次 DMA 写入一个新采样, 就转发一帧)
 * @note  在 while(1) 中持续调用。通过对比 DMA 写指针，检测新到达的采样，
 *        每帧只携带 1 个 ADC 值。若 CAN TX FIFO 满则本轮退出，数据仍保留在
 *        adc_buf 中，下一轮自动补发，不丢数据。
 */
void ADC_Stream_Forward(void)
{
  /* 1. 读取 DMA 剩余计数，换算当前写入索引 */
  uint32_t ndtr    = __HAL_DMA_GET_COUNTER(&hdma_adc1);
  uint32_t cur_pos = SA_ADC_BUF_SIZE - ndtr;          /* [0, SA_ADC_BUF_SIZE] */

  /* 2. 计算自上次发送以来新增的采样点数（处理环形缓冲区回绕） */
  uint32_t count;
  if (cur_pos >= s_adc_stream_last_pos)
    count = cur_pos - s_adc_stream_last_pos;
  else
    count = SA_ADC_BUF_SIZE - s_adc_stream_last_pos + cur_pos;

  while (count > 0)
  {
    uint16_t sample = adc_buf[s_adc_stream_last_pos];
	uint16_t encoder_num = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    memcpy(&CAN_SLAVE_ADC_STREAM[2], &sample,
					sizeof(uint16_t));
	memcpy(&CAN_SLAVE_ADC_STREAM[4], &encoder_num,
					sizeof(uint16_t));

    // /* 发送成功才推进指针；FIFO 满(HAL_ERROR)则退出，下轮补发 */
    if (FDCAN1_Send(CAN_SLAVE_ID, CAN_SLAVE_ADC_STREAM, 8) != HAL_OK)
      break;

    s_adc_stream_last_pos = (s_adc_stream_last_pos + 1) % SA_ADC_BUF_SIZE;
    count--;
    s_adc_stream_seq++;
  }
}

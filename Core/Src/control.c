#include "control.h"
#include "drv_can.h"
#include "main.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////	 
uint8_t CAN_SLAVE_ANSWER[8] = { 0x00, 0x20 };         // 从机应答指令
uint8_t CAN_SLAVE_AMPLITUDE[8] = { 0x00, 0x21 };  // 振幅返回
uint8_t CAN_SLAVE_PERIOD[8] = { 0x00, 0x22 };  // 周期返回
uint8_t CAN_SLAVE_PHASE_DIFF_1[8] = { 0x00, 0x23 };  // 相位差1返回
uint8_t CAN_SLAVE_PHASE_DIFF_2[8] = { 0x00, 0x24 };  // 相位差2返回

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

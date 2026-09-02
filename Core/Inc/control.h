#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

extern uint8_t CAN_SLAVE_ANSWER[8];
extern uint8_t CAN_SLAVE_AMPLITUDE[8];
extern uint8_t CAN_SLAVE_PERIOD[8];
extern uint8_t CAN_SLAVE_PHASE_DIFF_1[8];
extern uint8_t CAN_SLAVE_PHASE_DIFF_2[8];

void Control_Init(void);

void Control_CanRxCallback(uint32_t ID, uint8_t *buf, uint8_t len);

#endif
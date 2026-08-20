#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"



void Control_Init(void);

void Control_CanRxCallback(uint32_t ID, uint8_t *buf, uint8_t len);

#endif
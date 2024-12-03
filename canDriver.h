#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "TM4C123.h"

void CAN0_Init(void);
void CAN_TX(unsigned char data);
void CAN_RX_init(void);
uint8_t CAN_RX(void);

#endif
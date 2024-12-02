
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "TM4C123.h"

// Function prototypes
void CAN0_Init(void);
void CAN0_Transmit_Set(int objNo);
void CAN0_Receive_Set(int objNo);
void CAN0_Transmit_Data(int objNo, char data[8]);
char* CAN0_Receive_Data(int objNo);

#endif // CAN_DRIVER_H
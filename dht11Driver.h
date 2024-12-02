
#ifndef DHT11_DRIVER_H
#define DHT11_DRIVER_H

#include "TM4C123.h"

// DHT11 definitions
#define DHT (0)
#define MASK(x) (1UL<<(x))

// Function prototypes
uint8_t DHT11_Init(void);
uint8_t DHT11_Read_Data(void);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Check(void);
void DHT11_Rst(void);

#endif // DHT11_DRIVER_H
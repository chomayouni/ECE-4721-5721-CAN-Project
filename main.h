#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123.h"
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h"

// Function prototypes
void Pwm_Init(void);
void Timer_Init(void);
void TIMER1A_Handler(void);
void Uart_Init(void);
void uart_send_data(uint16_t Data);

#endif // MAIN_H
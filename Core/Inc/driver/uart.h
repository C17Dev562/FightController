#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx_hal.h"
#include "main.h"
void uart_init(void);
extern UART_HandleTypeDef UART1_Handler;

#endif

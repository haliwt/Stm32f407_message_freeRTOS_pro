#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define DEBUG_USARTx                                 USART2//USART1 //WT.EDIT
#define DEBUG_USARTx_BAUDRATE                        115200
#define DEBUG_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define DEBUG_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define DEBUG_USARTx_Tx_GPIO_PIN                     GPIO_PIN_5 //WT.EDIT  TXD =PD5
#define DEBUG_USARTx_Tx_GPIO                         GPIOD
#define DEBUG_USARTx_Rx_GPIO_PIN                     GPIO_PIN_6 //WT.EDIT RXD=PD6
#define DEBUG_USARTx_Rx_GPIO                         GPIOD

#define DEBUG_USARTx_AFx                             GPIO_AF7_USART2

#define DEBUG_USART_IRQn                             USART2_IRQn
/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husart_debug;

/* 函数声明 ------------------------------------------------------------------*/
void MX_DEBUG_USART_Init(void);


#endif  /* __BSP_DEBUG_USART_H__ */



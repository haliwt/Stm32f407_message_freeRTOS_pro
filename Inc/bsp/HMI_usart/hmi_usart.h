#ifndef __HMI_USARTX_H__
#define __HMI_USARTX_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 宏定义 --------------------------------------------------------------------*/
#define USARTx                                 USART3
#define USARTx_BAUDRATE                        115200
#define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
#define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

#define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_Tx_GPIO_PIN                     GPIO_PIN_8
#define USARTx_Tx_GPIO                         GPIOD
#define USARTx_Rx_GPIO_PIN                     GPIO_PIN_9   
#define USARTx_Rx_GPIO                         GPIOD

#define USARTx_AFx                             GPIO_AF7_USART3

#define USARTx_IRQHANDLER                      USART3_IRQHandler
#define USARTx_IRQn                            USART3_IRQn


/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx;

/* 函数声明 ------------------------------------------------------------------*/
void MX_USARTx_Init(void);


#endif  /* __BSP_USARTX_H__ */

#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
/********************通用定时器TIM参数定义，TIM2************/
#define GENERAL_TIMx                     TIM2
#define GENERAL_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM2_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM2_CLK_DISABLE()
#define GENERAL_TIM_IRQ                  TIM2_IRQn
#define GENERAL_TIM_INT_FUN              TIM2_IRQHandler

// 定义定时器预分频，定时器实际时钟频率为：84MHz/（GENERAL_TIM_PRESCALER+1）
#define GENERAL_TIM_PRESCALER           83  // 实际时钟频率为：1MHz
// 定义定时器周期，当定时器开始计数到BASIC_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD              1000  // 定时器产生中断频率为：1MHz/1000=1KHz，即1ms定时周期

// 最终定时器频率计算为： 84MHz/（GENERAL_TIM_PRESCALER+1）/GENERAL_TIM_PERIOD
// 比如需要产生20ms周期定时，可以设置为： 84MHz/（83+1）/1000=1Hz，即1ms周期
// 这里设置 GENERAL_TIM_PRESCALER=83；GENERAL_TIM_PERIOD=1000；

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;

/* 函数声明 ------------------------------------------------------------------*/

void GENERAL_TIMx_Init(void);

#endif	/* __GENERAL_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

#ifndef __Z_AXIS_H__
#define __Z_AXIS_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"



/* 宏定义 --------------------------------------------------------------------*/
//#define STEPMOTOR_TIM8_Z                        TIM8 //WT.EDIT
//#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
//#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_IT_CC3                  TIM_IT_CC3//TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CC3                TIM_FLAG_CC3//TIM_FLAG_CC1
//#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_Z_IRQHandler           TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_3               TIM_CHANNEL_3 //WT.EDIT
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE_Z()       __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机驱动器
#define STEPMOTOR_TIM_PUL_PORT_Z               GPIOI  //WT.EDIT                          // 对应驱动器的PUL-（驱动器使用共阳接法）
#define STEPMOTOR_TIM_PUL_PIN_Z                GPIO_PIN_7                       // 而PLU+直接接开发板的VCC
#define GPIO_AFx_TIMx                         GPIO_AF3_TIM8


#define STEPMOTOR_DIR_GPIO_CLK_ENABLE_Z()       __HAL_RCC_GPIOG_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR_PORT_Z                    GPIOG    //WT.EDIT                        // 对应驱动器的DIR-（驱动器使用共阳接法）
#define STEPMOTOR_DIR_PIN_Z                     GPIO_PIN_1                       // 而DIR+直接接开发板的VCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // 引脚不作为复用功能使用

#define STEPMOTOR_ENA_GPIO_CLK_ENABLE_Z()       __HAL_RCC_GPIOE_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA_PORT_Z                   GPIOE  //WT.EDIT                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define STEPMOTOR_ENA_PIN_Z                     GPIO_PIN_7                       // 而ENA+直接接开发板的VCC

#define STEPMOTOR_DIR_FORWARD_Z()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_Z,STEPMOTOR_DIR_PIN_Z,GPIO_PIN_RESET)
#define STEPMOTOR_DIR_REVERSAL_Z()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_Z,STEPMOTOR_DIR_PIN_Z,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT_ENABLE_Z()             HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_Z,STEPMOTOR_ENA_PIN_Z,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT_DISABLE_Z()            HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_Z,STEPMOTOR_ENA_PIN_Z,GPIO_PIN_SET)




/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR_Z;
/* 函数声明 ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Z_Init(void);
void STEPMOTOR_AxisMoveRel_Z(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

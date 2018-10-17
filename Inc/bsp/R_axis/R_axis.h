#ifndef __R_AXIS_H__
#define __R_AXIS_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 宏定义 --------------------------------------------------------------------*/
//#define STEPMOTOR_TIM8_R                      TIM8 //WT.EDIT
//#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
//#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_IT_CC4                  TIM_IT_CC4//TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CC4                TIM_FLAG_CC4//TIM_FLAG_CC1
//#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_R_IRQHandler             TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_4               TIM_CHANNEL_4 //WT.EDIT
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE_R()       __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机驱动器
#define STEPMOTOR_TIM_PUL_PORT_R                GPIOI                            // 对应驱动器的PUL-（驱动器使用共阳接法）
#define STEPMOTOR_TIM_PUL_PIN_R                 GPIO_PIN_2                       // 而PLU+直接接开发板的VCC
#define GPIO_AFx_TIMx                         GPIO_AF3_TIM8


#define STEPMOTOR_DIR_GPIO_CLK_ENABLE_R()       __HAL_RCC_GPIOF_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR_PORT_R                    GPIOF    //WT.EDIT                        // 对应驱动器的DIR-（驱动器使用共阳接法）
#define STEPMOTOR_DIR_PIN_R                     GPIO_PIN_15                       // 而DIR+直接接开发板的VCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // 引脚不作为复用功能使用

#define STEPMOTOR_ENA_GPIO_CLK_ENABLE_R()       __HAL_RCC_GPIOG_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA_PORT_R                    GPIOG //WT.EDIT                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define STEPMOTOR_ENA_PIN_R                     GPIO_PIN_0                       // 而ENA+直接接开发板的VCC

#define STEPMOTOR_DIR_FORWARD_R()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_R,STEPMOTOR_DIR_PIN_R,GPIO_PIN_RESET)
#define STEPMOTOR_DIR_REVERSAL_R()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_R,STEPMOTOR_DIR_PIN_R,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT_ENABLE_R()             HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_R,STEPMOTOR_ENA_PIN_R,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT_DISABLE_R()            HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_R,STEPMOTOR_ENA_PIN_R,GPIO_PIN_SET)







/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR_R;
/* 函数声明 ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_R_Init(void);
void STEPMOTOR_AxisMoveRel_R(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	/* __STEPMOTOR_TIM_H__ */


#ifndef __R_AXIS_H__
#define __R_AXIS_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* �궨�� --------------------------------------------------------------------*/
//#define STEPMOTOR_TIM8_R                      TIM8 //WT.EDIT
//#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
//#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_IT_CC4                  TIM_IT_CC4//TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CC4                TIM_FLAG_CC4//TIM_FLAG_CC1
//#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_R_IRQHandler             TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_4               TIM_CHANNEL_4 //WT.EDIT
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE_R()       __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL_PORT_R                GPIOI                            // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL_PIN_R                 GPIO_PIN_2                       // ��PLU+ֱ�ӽӿ������VCC
#define GPIO_AFx_TIMx                         GPIO_AF3_TIM8


#define STEPMOTOR_DIR_GPIO_CLK_ENABLE_R()       __HAL_RCC_GPIOF_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR_PORT_R                    GPIOF    //WT.EDIT                        // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR_PIN_R                     GPIO_PIN_15                       // ��DIR+ֱ�ӽӿ������VCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // ���Ų���Ϊ���ù���ʹ��

#define STEPMOTOR_ENA_GPIO_CLK_ENABLE_R()       __HAL_RCC_GPIOG_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA_PORT_R                    GPIOG //WT.EDIT                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA_PIN_R                     GPIO_PIN_0                       // ��ENA+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR_FORWARD_R()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_R,STEPMOTOR_DIR_PIN_R,GPIO_PIN_RESET)
#define STEPMOTOR_DIR_REVERSAL_R()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_R,STEPMOTOR_DIR_PIN_R,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT_ENABLE_R()             HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_R,STEPMOTOR_ENA_PIN_R,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT_DISABLE_R()            HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_R,STEPMOTOR_ENA_PIN_R,GPIO_PIN_SET)







/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR_R;
/* �������� ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_R_Init(void);
void STEPMOTOR_AxisMoveRel_R(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	/* __STEPMOTOR_TIM_H__ */


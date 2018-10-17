#ifndef __Z_AXIS_H__
#define __Z_AXIS_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"



/* �궨�� --------------------------------------------------------------------*/
//#define STEPMOTOR_TIM8_Z                        TIM8 //WT.EDIT
//#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
//#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_IT_CC3                  TIM_IT_CC3//TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CC3                TIM_FLAG_CC3//TIM_FLAG_CC1
//#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_Z_IRQHandler           TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_3               TIM_CHANNEL_3 //WT.EDIT
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE_Z()       __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL_PORT_Z               GPIOI  //WT.EDIT                          // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL_PIN_Z                GPIO_PIN_7                       // ��PLU+ֱ�ӽӿ������VCC
#define GPIO_AFx_TIMx                         GPIO_AF3_TIM8


#define STEPMOTOR_DIR_GPIO_CLK_ENABLE_Z()       __HAL_RCC_GPIOG_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR_PORT_Z                    GPIOG    //WT.EDIT                        // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR_PIN_Z                     GPIO_PIN_1                       // ��DIR+ֱ�ӽӿ������VCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // ���Ų���Ϊ���ù���ʹ��

#define STEPMOTOR_ENA_GPIO_CLK_ENABLE_Z()       __HAL_RCC_GPIOE_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA_PORT_Z                   GPIOE  //WT.EDIT                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA_PIN_Z                     GPIO_PIN_7                       // ��ENA+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR_FORWARD_Z()               HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_Z,STEPMOTOR_DIR_PIN_Z,GPIO_PIN_RESET)
#define STEPMOTOR_DIR_REVERSAL_Z()              HAL_GPIO_WritePin(STEPMOTOR_DIR_PORT_Z,STEPMOTOR_DIR_PIN_Z,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT_ENABLE_Z()             HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_Z,STEPMOTOR_ENA_PIN_Z,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT_DISABLE_Z()            HAL_GPIO_WritePin(STEPMOTOR_ENA_PORT_Z,STEPMOTOR_ENA_PIN_Z,GPIO_PIN_SET)




/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR_Z;
/* �������� ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Z_Init(void);
void STEPMOTOR_AxisMoveRel_Z(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

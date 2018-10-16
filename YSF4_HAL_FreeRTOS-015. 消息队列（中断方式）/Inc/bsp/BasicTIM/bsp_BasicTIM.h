#ifndef __BASIC_TIM_H__
#define __BASIC_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
/********************������ʱ��TIM�������壬ֻ��TIM6 & TIM7************/

#define BASIC_TIMx                     TIM6
#define BASIC_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM6_CLK_ENABLE()
#define BASIC_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM6_CLK_DISABLE()
#define BASIC_TIM_IRQ                  TIM6_DAC_IRQn
#define BASIC_TIM_INT_FUN              TIM6_DAC_IRQHandler


// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��BASIC_TIMx_PRESCALER+1��
#define BASIC_TIMx_PRESCALER           0  // ʵ��ʱ��Ƶ��Ϊ��1MHz
// ���嶨ʱ�����ڣ�����ʱ����ʼ������BASIC_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define BASIC_TIMx_PERIOD              4200  // ��ʱ�������ж�Ƶ��Ϊ��1MHz/1000=1KHz����1ms��ʱ����

// ���ն�ʱ��Ƶ�ʼ���Ϊ�� 84MHz/��BASIC_TIMx_PRESCALER+1��/BASIC_TIMx_PERIOD
// ������Ҫ����20ms���ڶ�ʱ����������Ϊ�� 84MHz/��83+1��/1000=1Hz����1ms����
// �������� BASIC_TIMx_PRESCALER=83��BASIC_TIMx_PERIOD=1000��

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx;
/* �������� ------------------------------------------------------------------*/

void BASIC_TIMx_Init(void);

#endif	/* __BASIC_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

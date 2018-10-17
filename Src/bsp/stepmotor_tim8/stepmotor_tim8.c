#include "stepmotor_tim8/stepmotor_tim8.h"
#include "R_axis/r_axis.h" 
#include "Z_axis/z_axis.h"

/******************************************************************
  *
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  *
******************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIM8)
  {
    /* ������ʱ������ʱ��ʹ�� */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
}

/**************************************************************
  *
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  *
**************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIM8)
  {
    /* ������ʱ������ʱ�ӽ��� ---R_axis*/
    STEPMOTOR_TIM_RCC_CLK_DISABLE();
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT_R,STEPMOTOR_TIM_PUL_PIN_R);
    HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT_R,STEPMOTOR_DIR_PIN_R);
    HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT_R,STEPMOTOR_ENA_PIN_R);
	  
	  
	/* ������ʱ������ʱ�ӽ��� ----Z_axis*/
   // STEPMOTOR_TIM_RCC_CLK_DISABLE();
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT_Z,STEPMOTOR_TIM_PUL_PIN_Z);
    HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT_Z,STEPMOTOR_DIR_PIN_Z);
    HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT_Z,STEPMOTOR_ENA_PIN_Z);
    
    HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
  }
  
  
   
  
} 



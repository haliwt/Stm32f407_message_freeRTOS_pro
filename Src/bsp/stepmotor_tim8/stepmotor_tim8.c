#include "stepmotor_tim8/stepmotor_tim8.h"
#include "R_axis/r_axis.h" 
#include "Z_axis/z_axis.h"

/******************************************************************
  *
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  *
******************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIM8)
  {
    /* 基本定时器外设时钟使能 */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
}

/**************************************************************
  *
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  *
**************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIM8)
  {
    /* 基本定时器外设时钟禁用 ---R_axis*/
    STEPMOTOR_TIM_RCC_CLK_DISABLE();
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT_R,STEPMOTOR_TIM_PUL_PIN_R);
    HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT_R,STEPMOTOR_DIR_PIN_R);
    HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT_R,STEPMOTOR_ENA_PIN_R);
	  
	  
	/* 基本定时器外设时钟禁用 ----Z_axis*/
   // STEPMOTOR_TIM_RCC_CLK_DISABLE();
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL_PORT_Z,STEPMOTOR_TIM_PUL_PIN_Z);
    HAL_GPIO_DeInit(STEPMOTOR_DIR_PORT_Z,STEPMOTOR_DIR_PIN_Z);
    HAL_GPIO_DeInit(STEPMOTOR_ENA_PORT_Z,STEPMOTOR_ENA_PIN_Z);
    
    HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
  }
  
  
   
  
} 



/**
  ******************************************************************************
  * 文件名程: bsp_BasicTIM.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 基本定时器TIM6 & TIM7底层驱动程序
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "BasicTIM/bsp_BasicTIM.h" 

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx;
volatile uint32_t ulHighFrequencyTimerTicks;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 基本定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void BASIC_TIMx_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htimx.Instance = BASIC_TIMx;
  htimx.Init.Prescaler = BASIC_TIMx_PRESCALER;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = BASIC_TIMx_PERIOD;
  HAL_TIM_Base_Init(&htimx);

  sMasterConfig.MasterOutputTrigger = TIM_IT_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig);
 
  /* 在中断模式下启动定时器 */
  HAL_TIM_Base_Start_IT(&htimx);
}

/**
  * 函数功能: 基本定时器硬件初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BASIC_TIMx)
  {
    /* 基本定时器外设时钟使能 */
    BASIC_TIM_RCC_CLK_ENABLE();

    /* 外设中断配置 */
    HAL_NVIC_SetPriority(BASIC_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(BASIC_TIM_IRQ);
  }
}

/**
  * 函数功能: 基本定时器硬件反初始化配置
  * 输入参数: htim_base：基本定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==BASIC_TIMx)
  {
    /* 基本定时器外设时钟禁用 */
    BASIC_TIM_RCC_CLK_DISABLE();

    /* 关闭外设中断 */
    HAL_NVIC_DisableIRQ(BASIC_TIM_IRQ);
  }
} 

/**
  * 函数功能: 非阻塞模式下定时器的回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(__HAL_TIM_GET_IT_SOURCE(&htimx,TIM_IT_UPDATE)!=RESET)
  {
    ulHighFrequencyTimerTicks++;
    __HAL_TIM_CLEAR_IT(&htimx, TIM_IT_UPDATE);
  }
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

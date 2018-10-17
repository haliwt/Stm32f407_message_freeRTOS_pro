#ifndef __STEPMOTOR_TIM8_H__
#define __STEPMOTOR_TIM8_H__

#include "stm32f4xx_hal.h"


/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  // 电机旋转状态
  __IO uint8_t  dir ;        // 电机旋转方向
  __IO int32_t  step_delay;  // 下个脉冲周期（时间间隔），启动时为加速度
  __IO uint32_t decel_start; // 启动减速位置
  __IO int32_t  decel_val;   // 减速阶段步数
  __IO int32_t  min_delay;   // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; // 加减速阶段计数值
}speedRampData;


#define STEPMOTOR_TIM8                         TIM8 //WT.EDIT
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn




//Z轴
// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER_Z               5  // 步进电机驱动器细分设置为：   32  细分
//#define STEPMOTOR_TIM_PRESCALER               9  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               19  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               39  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               79  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               159 // 步进电机驱动器细分设置为：   1  细分
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// 定义高级定时器重复计数寄存器值
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    0 // 顺时针
#define CCW                                   1 // 逆时针

#define STOP                                  0 // 加减速曲线状态：停止
#define ACCEL                                 1 // 加减速曲线状态：加速阶段
#define DECEL                                 2 // 加减速曲线状态：减速阶段
#define RUN                                   3 // 加减速曲线状态：匀速阶段
#define T1_FREQ_Z                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER_Z+1)) // 频率ft值
#define FSPR                                  200         //步进电机单圈步数
#define MICRO_STEP                            32          // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数

// 数学常数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr
#define A_T_x10_Z                             ((float)(10*ALPHA*T1_FREQ_Z))
#define T1_FREQ_148_Z                         ((float)((T1_FREQ_Z*0.676)/10)) // 0.676为误差修正值
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))


//R轴
// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER_R               5  // 步进电机驱动器细分设置为：   32  细分
//#define STEPMOTOR_TIM_PRESCALER               9  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               19  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               39  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               79  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               159 // 步进电机驱动器细分设置为：   1  细分
// 定义定时器周期，输出比较模式周期设置为0xFFFF
//#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// 定义高级定时器重复计数寄存器值
//#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

//#define FALSE                                 0
//#define TRUE                                  1
//#define CW                                    0 // 顺时针
//#define CCW                                   1 // 逆时针

//#define STOP                                  0 // 加减速曲线状态：停止
//#define ACCEL                                 1 // 加减速曲线状态：加速阶段
//#define DECEL                                 2 // 加减速曲线状态：减速阶段
//#define RUN                                   3 // 加减速曲线状态：匀速阶段
#define T1_FREQ_R                             (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER_R+1)) // 频率ft值
//#define FSPR                                  200         //步进电机单圈步数
//#define MICRO_STEP                            32          // 步进电机驱动器细分数
//#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数

// 数学常数
//#define ALPHA                                  ((float)(2*3.14159/SPR))       // α= 2*pi/spr
#define A_T_x10_R                              ((float)(10*ALPHA*T1_FREQ_R))
#define T1_FREQ_148_R                          ((float)((T1_FREQ_R*0.676)/10)) // 0.676为误差修正值
#define A_SQ_R                                  ((float)(2*100000*ALPHA)) 
//#define A_x200                                 ((float)(200*ALPHA))


#endif


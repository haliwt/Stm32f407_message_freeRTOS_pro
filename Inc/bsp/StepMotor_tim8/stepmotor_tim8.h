#ifndef __STEPMOTOR_TIM8_H__
#define __STEPMOTOR_TIM8_H__

#include "stm32f4xx_hal.h"


/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  // �����ת״̬
  __IO uint8_t  dir ;        // �����ת����
  __IO int32_t  step_delay;  // �¸��������ڣ�ʱ������������ʱΪ���ٶ�
  __IO uint32_t decel_start; // ��������λ��
  __IO int32_t  decel_val;   // ���ٽ׶β���
  __IO int32_t  min_delay;   // ��С��������(����ٶȣ������ٶ��ٶ�)
  __IO int32_t  accel_count; // �Ӽ��ٽ׶μ���ֵ
}speedRampData;


#define STEPMOTOR_TIM8                         TIM8 //WT.EDIT
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn




//Z��
// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define STEPMOTOR_TIM_PRESCALER_Z               5  // �������������ϸ������Ϊ��   32  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               9  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               19  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               39  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               79  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               159 // �������������ϸ������Ϊ��   1  ϸ��
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    0 // ˳ʱ��
#define CCW                                   1 // ��ʱ��

#define STOP                                  0 // �Ӽ�������״̬��ֹͣ
#define ACCEL                                 1 // �Ӽ�������״̬�����ٽ׶�
#define DECEL                                 2 // �Ӽ�������״̬�����ٽ׶�
#define RUN                                   3 // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_Z                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER_Z+1)) // Ƶ��ftֵ
#define FSPR                                  200         //���������Ȧ����
#define MICRO_STEP                            32          // �������������ϸ����
#define SPR                                   (FSPR*MICRO_STEP)   // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA                                 ((float)(2*3.14159/SPR))       // ��= 2*pi/spr
#define A_T_x10_Z                             ((float)(10*ALPHA*T1_FREQ_Z))
#define T1_FREQ_148_Z                         ((float)((T1_FREQ_Z*0.676)/10)) // 0.676Ϊ�������ֵ
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))


//R��
// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define STEPMOTOR_TIM_PRESCALER_R               5  // �������������ϸ������Ϊ��   32  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               9  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               19  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               39  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               79  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               159 // �������������ϸ������Ϊ��   1  ϸ��
// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
//#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
//#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

//#define FALSE                                 0
//#define TRUE                                  1
//#define CW                                    0 // ˳ʱ��
//#define CCW                                   1 // ��ʱ��

//#define STOP                                  0 // �Ӽ�������״̬��ֹͣ
//#define ACCEL                                 1 // �Ӽ�������״̬�����ٽ׶�
//#define DECEL                                 2 // �Ӽ�������״̬�����ٽ׶�
//#define RUN                                   3 // �Ӽ�������״̬�����ٽ׶�
#define T1_FREQ_R                             (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER_R+1)) // Ƶ��ftֵ
//#define FSPR                                  200         //���������Ȧ����
//#define MICRO_STEP                            32          // �������������ϸ����
//#define SPR                                   (FSPR*MICRO_STEP)   // ��תһȦ��Ҫ��������

// ��ѧ����
//#define ALPHA                                  ((float)(2*3.14159/SPR))       // ��= 2*pi/spr
#define A_T_x10_R                              ((float)(10*ALPHA*T1_FREQ_R))
#define T1_FREQ_148_R                          ((float)((T1_FREQ_R*0.676)/10)) // 0.676Ϊ�������ֵ
#define A_SQ_R                                  ((float)(2*100000*ALPHA)) 
//#define A_x200                                 ((float)(200*ALPHA))


#endif


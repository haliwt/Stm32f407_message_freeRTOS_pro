#include "Z_axis/z_axis.h" 
#include "stepmotor_tim8/stepmotor_tim8.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR_Z;
speedRampData srd_z              = {STOP,CW,0,0,0,0,0};         // �Ӽ������߱���
__IO int32_t  step_position_z     = 0;           // ��ǰλ��
__IO uint8_t  MotionStatus_z      = 0;           //�Ƿ����˶���0��ֹͣ��1���˶�

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Z_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* ���Ŷ˿�ʱ��ʹ�� */
  STEPMOTOR_TIM_GPIO_CLK_ENABLE_Z();
  STEPMOTOR_DIR_GPIO_CLK_ENABLE_Z();
  STEPMOTOR_ENA_GPIO_CLK_ENABLE_Z();
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL_PIN_Z;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL_PORT_Z, &GPIO_InitStruct);
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR_PIN_Z;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;                    // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR_PORT_Z, &GPIO_InitStruct);
  
  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA_PIN_Z;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;                     // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA_PORT_Z, &GPIO_InitStruct);
  
  STEPMOTOR_DIR_FORWARD_Z();//STEPMOTOR_DIR_FORWARD_Z();
  STEPMOTOR_OUTPUT_DISABLE_Z();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Z_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  
  STEPMOTOR_TIM_RCC_CLK_ENABLE();
  
  /* STEPMOTOR���GPIO��ʼ������ */
  STEPMOTOR_GPIO_Z_Init();
  
  /* ��ʱ�������������� */
  htimx_STEPMOTOR_Z.Instance = STEPMOTOR_TIM8;                          // ��ʱ�����
  htimx_STEPMOTOR_Z.Init.Prescaler = STEPMOTOR_TIM_PRESCALER_Z;           // ��ʱ��Ԥ��Ƶ��
  htimx_STEPMOTOR_Z.Init.CounterMode = TIM_COUNTERMODE_UP;              // �����������ϼ���
  htimx_STEPMOTOR_Z.Init.Period = STEPMOTOR_TIM_PERIOD;                 // ��ʱ������
  htimx_STEPMOTOR_Z.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;          // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htimx_STEPMOTOR_Z);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       		// ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR_Z, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                            // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;           // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR_Z, &sConfigOC, STEPMOTOR_TIM_CHANNEL_3);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(STEPMOTOR_TIM8, STEPMOTOR_TIM_CHANNEL_3, TIM_CCx_DISABLE);
  
  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);
  
  __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR_Z, STEPMOTOR_TIM_FLAG_CC3);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htimx_STEPMOTOR_Z, STEPMOTOR_TIM_IT_CC3);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR_Z);  
  HAL_TIM_Base_Start(&htimx_STEPMOTOR_Z);// ʹ�ܶ�ʱ��
}



/********************************************************************************************
  *
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: step���ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
              accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2
              decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
              speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ�����������ȼ��ٵ�����ٶȣ�Ȼ���ں���λ�ÿ�ʼ
  *           ������ֹͣ��ʹ�������˶�����Ϊָ���Ĳ���������Ӽ��ٽ׶κ̲ܶ���
  *           �ٶȺ������ǻ�û�ﵽ����ٶȾ�Ҫ��ʼ����
  *
**********************************************************************************************/
void STEPMOTOR_AxisMoveRel_Z( int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{  
  __IO uint16_t tim_count;
  // �ﵽ����ٶ�ʱ�Ĳ���
  __IO uint32_t max_s_lim;
  // ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
  __IO uint32_t accel_lim;

  if(MotionStatus_z != STOP)    // ֻ�����������ֹͣ��ʱ��ż���
    return;
  if(step < 0) // ����Ϊ����
  {
    srd_z.dir = CCW; // ��ʱ�뷽����ת
    STEPMOTOR_DIR_REVERSAL_Z();
    step =-step;   // ��ȡ��������ֵ
  }
  else
  {
    srd_z.dir = CW; // ˳ʱ�뷽����ת
    STEPMOTOR_DIR_FORWARD_Z();
  }
  
  if(step == 1)    // ����Ϊ1
  {
    srd_z.accel_count = -1;   // ֻ�ƶ�һ��
    srd_z.run_state = DECEL;  // ����״̬.
    srd_z.step_delay = 1000;	// ����ʱ	
  }
  else if(step != 0)  // ���Ŀ���˶�������Ϊ0
  {
    // ���ǵĵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    // min_delay = (alpha / tt)/ w
    srd_z.min_delay = (int32_t)(A_T_x10_Z/speed);

    // ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    srd_z.step_delay = (int32_t)((T1_FREQ_148_Z * sqrt(A_SQ / accel))/10);

    // ������ٲ�֮��ﵽ����ٶȵ�����
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
    // ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // ������ٲ�֮�����Ǳ��뿪ʼ����
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (uint32_t)(step*decel/(accel+decel));
    // ���Ǳ����������1�����ܲ��ܿ�ʼ����.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // ʹ�������������ǿ��Լ�������ٽ׶β���
    if(accel_lim <= max_s_lim){
      srd_z.decel_val = accel_lim - step;
    }
    else{
      srd_z.decel_val = -(max_s_lim*accel/decel);
    }
    // ��ֻʣ��һ�����Ǳ������
    if(srd_z.decel_val == 0){
      srd_z.decel_val = -1;
    }

    // ���㿪ʼ����ʱ�Ĳ���
    srd_z.decel_start = step + srd_z.decel_val;

    // �������ٶȺ��������ǾͲ���Ҫ���м����˶�
    if(srd_z.step_delay <= srd_z.min_delay){
      srd_z.step_delay = srd_z.min_delay;
      srd_z.run_state = RUN;
    }
    else{
      srd_z.run_state = ACCEL;
    }    
    // ��λ���ٶȼ���ֵ
    srd_z.accel_count = 0;
  }
  MotionStatus_z = 1; // ���Ϊ�˶�״̬
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR_Z);
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR_Z,STEPMOTOR_TIM_CHANNEL_3,tim_count+srd_z.step_delay); // ���ö�ʱ���Ƚ�ֵ
  TIM_CCxChannelCmd(STEPMOTOR_TIM8, STEPMOTOR_TIM_CHANNEL_3, TIM_CCx_ENABLE);// ʹ�ܶ�ʱ��ͨ�� 
  STEPMOTOR_OUTPUT_ENABLE_Z();
}


/**
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void STEPMOTOR_TIMx_Z_IRQHandler(void)//��ʱ���жϴ���
{ 
  __IO uint32_t tim_count=0;
  __IO uint32_t tmp = 0;
  // �����£��£�һ����ʱ����
  uint16_t new_step_delay=0;
  // ���ٹ��������һ����ʱ���������ڣ�.
  __IO static uint16_t last_accel_delay=0;
  // ���ƶ�����������
  __IO static uint32_t step_count = 0;
  // ��¼new_step_delay�е������������һ������ľ���
  __IO static int32_t rest = 0;
  //��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
  __IO static uint8_t i=0;
  
  if(__HAL_TIM_GET_IT_SOURCE(&htimx_STEPMOTOR_Z, STEPMOTOR_TIM_IT_CC3) !=RESET)
  {
    // �����ʱ���ж�
    __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR_Z, STEPMOTOR_TIM_IT_CC3);
    
    // ���ñȽ�ֵ
    tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR_Z);
    tmp = tim_count+srd_z.step_delay;
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR_Z,STEPMOTOR_TIM_CHANNEL_3,tmp);

    i++;     // ��ʱ���жϴ�������ֵ
    if(i==2) // 2�Σ�˵���Ѿ����һ����������
    {
      i=0;   // ���㶨ʱ���жϴ�������ֵ
      switch(srd_z.run_state) // �Ӽ������߽׶�
      {
        case STOP:
          step_count = 0;  // ���㲽��������
          rest = 0;        // ������ֵ
          // �ر�ͨ��
          TIM_CCxChannelCmd(STEPMOTOR_TIM8, STEPMOTOR_TIM_CHANNEL_3, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR_Z, STEPMOTOR_TIM_FLAG_CC3);
          STEPMOTOR_OUTPUT_DISABLE_Z();
          MotionStatus_z = 0;  //  ���Ϊֹͣ״̬     
          break;

        case ACCEL:
          step_count++;      // ������1
          if(srd_z.dir==CW)
          {	  	
            step_position_z++; // ����λ�ü�1
          }
          else
          {
            step_position_z--; // ����λ�ü�1
          }
          srd_z.accel_count++; // ���ټ���ֵ��1
          new_step_delay = srd_z.step_delay - (((2 *srd_z.step_delay) + rest)/(4 * srd_z.accel_count + 1));//������(��)һ����������(ʱ����)
          rest = ((2 * srd_z.step_delay)+rest)%(4 * srd_z.accel_count + 1);// �����������´μ��㲹���������������
          if(step_count >= srd_z.decel_start)// ����ǹ�Ӧ�ÿ�ʼ����
          {
            srd_z.accel_count = srd_z.decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
            srd_z.run_state = DECEL;           // �¸����������ٽ׶�
          }
          else if(new_step_delay <= srd_z.min_delay) // ����Ƿ񵽴�����������ٶ�
          {
            last_accel_delay = new_step_delay; // ������ٹ��������һ����ʱ���������ڣ�
            new_step_delay = srd_z.min_delay;    // ʹ��min_delay����Ӧ����ٶ�speed��
            rest = 0;                          // ������ֵ
            srd_z.run_state = RUN;               // ����Ϊ��������״̬
          }
          break;

        case RUN:
          step_count++;  // ������1
          if(srd_z.dir==CW)
          {	  	
            step_position_z++; // ����λ�ü�1
          }
          else
          {
            step_position_z--; // ����λ�ü�1
          }
          new_step_delay = srd_z.min_delay;     // ʹ��min_delay����Ӧ����ٶ�speed��
          if(step_count >= srd_z.decel_start)   // ��Ҫ��ʼ����
          {
            srd_z.accel_count = srd_z.decel_val;  // ���ٲ�����Ϊ���ټ���ֵ
            new_step_delay = last_accel_delay;// �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
            srd_z.run_state = DECEL;            // ״̬�ı�Ϊ����
          }
          break;

        case DECEL:
          step_count++;  // ������1
          if(srd_z.dir==CW)
          {	  	
            step_position_z++; // ����λ�ü�1
          }
          else
          {
            step_position_z--; // ����λ�ü�1
          }
          srd_z.accel_count++;
          new_step_delay = srd_z.step_delay - (((2 * srd_z.step_delay) + rest)/(4 * srd_z.accel_count + 1)); //������(��)һ����������(ʱ����)
          rest = ((2 * srd_z.step_delay)+rest)%(4 * srd_z.accel_count + 1);// �����������´μ��㲹���������������
          
          //����Ƿ�Ϊ���һ��
          if(srd_z.accel_count >= 0)
          {
            srd_z.run_state = STOP;
          }
          break;
      }      
      srd_z.step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
    }
  }
}


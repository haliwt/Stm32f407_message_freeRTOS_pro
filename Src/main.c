#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "led/bsp_led.h"
//#include "R_axis/r_axis.h" 
#include "usart/bsp_debug_usart.h"
#include "key/bsp_key.h"
//#include "Z_axis/z_axis.h"
#include "StepMotor/bsp_StepMotor.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
static TaskHandle_t xHandleTaskUserIF = NULL;
static TaskHandle_t xHandleTaskX_axis = NULL;
static TaskHandle_t xHandleTaskY_axis = NULL;
static TaskHandle_t xHandleTaskZ_axis = NULL;
static TaskHandle_t xHandleTaskR_axis = NULL;
static QueueHandle_t xQueue1 = NULL;
static QueueHandle_t xQueue2 = NULL;
static QueueHandle_t xQueue3 = NULL; //WT.EDIT
static QueueHandle_t xQueue4 = NULL; //WT.EDIT

KEY key1,key2,key3,key4,key5;

uint8_t aRxBuffer[8];
// �ٶ����ֵ���������͵����������Щ�����1800����Щ���Դﵽ4000
__IO uint32_t set_speed  = 3000;         // �ٶ� ��λΪ0.05rad/sec
// ���ٶȺͼ��ٶ�ѡȡһ�����ʵ����Ҫ��ֵԽ���ٶȱ仯Խ�죬�Ӽ��ٽ׶αȽ϶���
// ���Լ��ٶȺͼ��ٶ�ֵһ������ʵ��Ӧ���жೢ�Գ����Ľ��
__IO uint32_t step_accel = 100;         // ���ٶ� ��λΪ0.0.5rad/sec^2
__IO uint32_t step_decel = 100;         // ���ٶ� ��λΪ0.05rad/sec^2

typedef struct Msg
{
	uint8_t  ucMessageID;
	uint16_t usData[2];
	uint32_t ulData[2];
}MSG_T;

MSG_T  g_tMsg; /* ����һ���ṹ��������Ϣ���� */

/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint16_t CCR1_Val;

/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void vTaskTaskUserIF(void *pvParameters);
static void vTaskX_axis(void *pvParameters);
static void vTaskY_axis(void *pvParameters);
static void vTaskZ_axis(void *pvParameters);
static void vTaskR_axis(void *pvParameters);  //WT.EDIT 
static void AppTaskCreate (void);
static void AppObjCreate (void);

/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  
  MX_DEBUG_USART_Init();

   /* ��ʼ��LED */
  LED_GPIO_Init();
  /* ���Ӱ�����ʼ�� */
  KEY_GPIO_Init();
  /* ������ʱ����ʼ����100us�ж�һ�� */
	
	 STEPMOTOR_TIMx_Init();
	/* �������� */
	AppTaskCreate();
  /* ��������ͨ�Ż��� */
	AppObjCreate();	
  /* �������ȣ���ʼִ������ */
  vTaskStartScheduler();
  
  /* ����ѭ�� */
  while (1)
  {
  }
}

/**
  * ��������: �ӿ���Ϣ����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static uint32_t g_uiCount = 0; /* ����Ϊȫ�־�̬�������������ݸ��� */

static void vTaskTaskUserIF(void *pvParameters)
{
  uint8_t pcWriteBuffer[500];
    /* �������� */
    KeyCreate(&key1,GetPinStateOfKey1);
	KeyCreate(&key2,GetPinStateOfKey2);
	KeyCreate(&key3,GetPinStateOfKey3);
 
  printf("KEY1��KEY2��KEY3��Ӧ��ͬ�������\n");

    while(1)
    {
      Key_RefreshState(&key1);//ˢ�°���״̬
      Key_RefreshState(&key2);//ˢ�°���״̬
      Key_RefreshState(&key3);//ˢ�°���״̬
   
      #if 0
	  HAL_UART_Receive(&husart_debug,aRxBuffer,8,0xffff);
	   printf("aRxBuffer[0]=%#x \n",aRxBuffer[0]);
	   printf("aRxBuffer[1]=%#x \n",aRxBuffer[1]);
	   printf("aRxBuffer[2]=%#x \n",aRxBuffer[2]);
	   printf("aRxBuffer[7]=%#x \n",aRxBuffer[7]);
	   #endif 
      if(Key_AccessTimes(&key1,KEY_ACCESS_READ)!=0)//���������¹�
      {
        printf("=================================================\r\n");
        printf("������      ����״̬ ���ȼ�   ʣ��ջ �������\r\n");
        vTaskList((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
       
        printf("\r\n������       ���м���         ʹ����\r\n");
        vTaskGetRunTimeStats((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
        Key_AccessTimes(&key1,KEY_ACCESS_WRITE_CLEAR);
      }
      
      if(Key_AccessTimes(&key2,KEY_ACCESS_READ)!=0)//���������¹�
      {         
          printf("KEY2���£��������ζ�ʱ���жϣ�50ms���ڶ�ʱ���жϸ�����vTaskMsgPro������Ϣ\r\n");
		   MSG_T   *ptMsg=NULL;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
         /* ��ʼ���ṹ��ָ�� */
          ptMsg = &g_tMsg;
          
		
		  
		  /* ��ʼ������ */
		  ptMsg->ucMessageID++;
		  ptMsg->ulData[0]++;
		  ptMsg->usData[0]++;
		  	  
		 
		   /* ����Ϣ���з����� */
			xQueueSendFromISR(xQueue2,
					  (void *)&g_tMsg,
					   &xHigherPriorityTaskWoken);
			  
			/* ����Ϣ���з����-�   --WT.EDIT */
			xQueueSendFromISR(xQueue4,
					  (void *)&g_tMsg,
					   &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			Key_AccessTimes(&key2,KEY_ACCESS_WRITE_CLEAR);
	  }
      
      if(Key_AccessTimes(&key3,KEY_ACCESS_READ)!=0)//���������¹�
      {         
         printf("KEY3���£��������ζ�ʱ���жϣ�50ms���ڶ�ʱ���жϸ�����vTaskMsgPro������Ϣ\r\n");
		 BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
         g_uiCount++;
      
		  /* ����Ϣ���з����� */
        xQueueSendFromISR(xQueue1,
                  (void *)&g_uiCount,
                  &xHigherPriorityTaskWoken);
				  
		 /* ����Ϣ���з����� */
        xQueueSendFromISR(xQueue3,
                  (void *)&g_uiCount,
                  &xHigherPriorityTaskWoken);

        /* ���xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_2);
        Key_AccessTimes(&key3,KEY_ACCESS_WRITE_CLEAR);
	   }
         
     vTaskDelay(20);
  }
		
}

/**
  * ��������: LED1����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void vTaskX_axis(void *pvParameters)
{
  BaseType_t xResult;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* �������ȴ�ʱ��Ϊ300ms */
  uint8_t ucQueueMsgValue;
	

  while(1)
  {
    xResult = xQueueReceive(xQueue1,                   /* ��Ϣ���о�� */
                            (void *)&ucQueueMsgValue,  		   /* �����ȡ���ǽṹ��ĵ�ַ */
                            (TickType_t)xMaxBlockTime);/* ��������ʱ�� */


    if(xResult == pdPASS)
    {
      /* �ɹ����գ���ͨ�����ڽ����ݴ�ӡ���� */
		printf("X_axis:ucQueueMsgValue = %d\r\n", ucQueueMsgValue);
	 // STEPMOTOR_DisMoveAbs(AXIS_X,400,step_accel,step_decel,set_speed);//X���ƶ���400mm��λ�� �ֶԾ���
	   STEPMOTOR_AxisMoveRel(AXIS_X,30*SPR*CCW,step_accel,step_decel,set_speed); // Z�ᷴ���ƶ�30Ȧ����Ծ���
	  
    }
    else
    {
      LED1_TOGGLE;

    }
  }
}

/***********************************************************
  *
  * ��������: LED2����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *
***********************************************************/
static void vTaskY_axis(void *pvParameters)
{
    MSG_T *ptMsg;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* �������ȴ�ʱ��Ϊ200ms */

  while(1)
  {
   xResult = xQueueReceive(xQueue2,                   /* ��Ϣ���о�� */
                          (void *)&ptMsg,  /* �洢���յ������ݵ�����ucQueueMsgValue�� */
                          (TickType_t)xMaxBlockTime);/* ��������ʱ�� */
  
    if(xResult == pdPASS)
    {
      
       /* �ɹ����գ���ͨ�����ڽ����ݴ�ӡ���� */
      printf("���յ���Ϣ��������ptMsg->ucMessageID = %d\r\n", ptMsg->ucMessageID);
      printf("���յ���Ϣ��������ptMsg->ulData[0] = %d\r\n", ptMsg->ulData[0]);
      printf("���յ���Ϣ��������ptMsg->usData[0] = %d\r\n", ptMsg->usData[0]);
	  STEPMOTOR_AxisMoveRel(AXIS_Y,30*SPR*CCW,step_accel,step_decel,set_speed); // X�ᷴ���ƶ�30Ȧ
		
	 
    }
    else
    {
      LED2_TOGGLE;
    }
	
  }
}

/**************************************************
  *
  * ��������: Z_axis����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *
***************************************************/
static void vTaskZ_axis(void *pvParameters)
{
   BaseType_t xResult;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* �������ȴ�ʱ��Ϊ300ms */
  uint8_t ucQueueMsgValue;
	

  while(1)
  {
    xResult = xQueueReceive(xQueue3,                   /* ��Ϣ���о�� */
                            (void *)&ucQueueMsgValue,  		   /* �����ȡ���ǽṹ��ĵ�ַ */
                            (TickType_t)xMaxBlockTime);/* ��������ʱ�� */


    if(xResult == pdPASS)
    {
      /* �ɹ����գ���ͨ�����ڽ����ݴ�ӡ���� */
		printf(" Z_axis:ucQueueMsgValue = %d\r\n", ucQueueMsgValue);
	  //STEPMOTOR_DisMoveAbs(AXIS_Z,400,step_accel,step_decel,set_speed);//X���ƶ���400mm��λ��
	   STEPMOTOR_AxisMoveRel(AXIS_Z,30*SPR*CCW,step_accel,step_decel,set_speed); // Z�ᷴ���ƶ�30Ȧ
	}
    else
    {
      LED3_TOGGLE;

    }
	
  }
}
/**************************************************
  *
  * ��������: R_axis����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *
***************************************************/
static void vTaskR_axis(void *pvParameters)
{
    while(1)
    {
       MSG_T *ptMsg;
	   BaseType_t xResult;
	   const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* �������ȴ�ʱ��Ϊ200ms */

	  while(1)
	  {
	   xResult = xQueueReceive(xQueue4,                   /* ��Ϣ���о�� */
							  (void *)&ptMsg,  /* �洢���յ������ݵ�����ucQueueMsgValue�� */
							  (TickType_t)xMaxBlockTime);/* ��������ʱ�� */
	  
		if(xResult == pdPASS)
		{
		  /* �ɹ����գ���ͨ�����ڽ����ݴ�ӡ���� */
			printf("R_axis=ptMsg->ucMessageID = %d\r\n", ptMsg->ucMessageID);
            printf("���յ���Ϣ��������ptMsg->ulData[0] = %d\r\n", ptMsg->ulData[0]);
            printf("���յ���Ϣ��������ptMsg->usData[0] = %d\r\n", ptMsg->usData[0]);
			STEPMOTOR_AxisMoveRel(AXIS_R,30*SPR*CW,step_accel,step_decel,set_speed);  // R�������ƶ�30Ȧ
	
		}
		else
		{
		  LED4_TOGGLE;
		}
	  }
	  
   }
}

/***************************************************
  *
  * ��������: ��������Ӧ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *
**************************************************/
static void AppTaskCreate (void)
{

    xTaskCreate( vTaskTaskUserIF,   	/* ������  */
                 "vTaskUserIF",     	/* ������    */
                 1024,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 1,                 	/* �������ȼ�*/
                 &xHandleTaskUserIF );  /* ������  */
	
    xTaskCreate( vTaskX_axis,   	      /* ������  */
                 "vTaskX_axis",     	  /* ������    */
                 1024,               	/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,              	/* �������  */
                 2,                 	/* �������ȼ�*/
                 &xHandleTaskX_axis );  /* ������  */
	
	
	xTaskCreate( vTaskY_axis,    		      /* ������  */
                 "vTaskY_axis",  		    /* ������    */
                 1024,         		    /* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,        		    /* �������  */
                 3,           		    /* �������ȼ�*/
                 &xHandleTaskY_axis);  /* ������  */
	
	xTaskCreate( vTaskZ_axis,     		    /* ������  */
                 "vTaskZ_axis",   		  /* ������    */
                 1024,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		  /* �������  */
                 4,               		/* �������ȼ�*/
                 &xHandleTaskZ_axis );  /* ������  */
	
	xTaskCreate( vTaskR_axis,     		    /* ������  */
                 "vTaskR_axis",   		  /* ������    */
                 1024,             		/* ����ջ��С����λword��Ҳ����4�ֽ� */
                 NULL,           		  /* �������  */
                 5,               		/* �������ȼ�*/
                 &xHandleTaskR_axis );  /* ������  */
	
}

/***************************************************
 *
 *�������ƣ�AppObjCreate(void)
 *�������ܣ�������Ϣ���С�
 *
 *
****************************************************/
static void AppObjCreate (void)
{
	/* ����10��uint8_t����Ϣ���� */
	xQueue1 = xQueueCreate(10, sizeof(uint8_t));
    if( xQueue1 == 0 )
    {
        /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
		printf("xQueue1 don't set up ERROR !\n");
    }
	
	/* ����10���洢ָ���������Ϣ���У�����CM3/CM4�ں���32λ����һ��ָ�����ռ��4���ֽ� */
	xQueue2 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue2 == 0 )
    {
        /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
		printf("xQueue2 don't set up ERROR !\n");
    }
	/* ����10���洢ָ���������Ϣ���У�����CM3/CM4�ں���32λ����һ��ָ�����ռ��4���ֽ� */
	xQueue3 = xQueueCreate(10, sizeof(uint8_t));
    if( xQueue3 == 0 )
    {
        /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
		printf("xQueue3 don't set up ERROR !\n");
    }
	xQueue4 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue4 == 0 )
    {
        printf("xQueue4 don't set up ERROR !\n");
    }
}



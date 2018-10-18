#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "led/bsp_led.h"
//#include "R_axis/r_axis.h" 
#include "usart/bsp_debug_usart.h"
#include "key/bsp_key.h"
//#include "Z_axis/z_axis.h"
#include "StepMotor/bsp_StepMotor.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
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
// 速度最大值由驱动器和电机决定，有些最大是1800，有些可以达到4000
__IO uint32_t set_speed  = 3000;         // 速度 单位为0.05rad/sec
// 加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
// 所以加速度和减速度值一般是在实际应用中多尝试出来的结果
__IO uint32_t step_accel = 100;         // 加速度 单位为0.0.5rad/sec^2
__IO uint32_t step_decel = 100;         // 减速度 单位为0.05rad/sec^2

typedef struct Msg
{
	uint8_t  ucMessageID;
	uint8_t usData[8];
	uint8_t ulData[8];
}MSG_T;

MSG_T  g_tMsg; /* 定义一个结构体用于消息队列 */

/* 扩展变量 ------------------------------------------------------------------*/


/* 私有函数原形 --------------------------------------------------------------*/
static void vTaskTaskUserIF(void *pvParameters);
static void vTaskX_axis(void *pvParameters);
static void vTaskY_axis(void *pvParameters);
static void vTaskZ_axis(void *pvParameters);
static void vTaskR_axis(void *pvParameters);  //WT.EDIT 
static void AppTaskCreate (void);
static void AppObjCreate (void);

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  
  MX_DEBUG_USART_Init();

   /* 初始化LED */
  LED_GPIO_Init();
  /* 板子按键初始化 */
  KEY_GPIO_Init();
  /* 基本定时器初始化：100us中断一次 */
	
	 STEPMOTOR_TIMx_Init();
	/* 创建任务 */
	AppTaskCreate();
  /* 创建任务通信机制 */
	AppObjCreate();	
  /* 启动调度，开始执行任务 */
  vTaskStartScheduler();
  
  /* 无限循环 */
  while (1)
  {
  }
}

/**
  * 函数功能: 接口消息处理
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static uint32_t g_uiCount = 0; /* 设置为全局静态变量，方便数据更新 */

static void vTaskTaskUserIF(void *pvParameters)
{
    uint8_t pcWriteBuffer[500];
	
    /* 创建按键 */
    KeyCreate(&key1,GetPinStateOfKey1);
	KeyCreate(&key2,GetPinStateOfKey2);
	KeyCreate(&key3,GetPinStateOfKey3);
 
  printf("KEY1、KEY2和KEY3对应不同任务情况\n");

    while(1)
    {
      Key_RefreshState(&key1);//刷新按键状态
      Key_RefreshState(&key2);//刷新按键状态
      Key_RefreshState(&key3);//刷新按键状态
   
      #if 1
	  HAL_UART_Receive(&husart_debug,aRxBuffer,8,0xffff);
	   printf("aRxBuffer[0]=%#x \n",aRxBuffer[0]);
	   printf("aRxBuffer[1]=%#x \n",aRxBuffer[1]);
	   printf("aRxBuffer[2]=%#x \n",aRxBuffer[2]);
	   printf("aRxBuffer[0]=%#x \n",aRxBuffer[3]);
	   printf("aRxBuffer[1]=%#x \n",aRxBuffer[4]);
	   printf("aRxBuffer[2]=%#x \n",aRxBuffer[5]);
	   printf("aRxBuffer[2]=%#x \n",aRxBuffer[6]);
	   printf("aRxBuffer[7]=%#x \n",aRxBuffer[7]);
	   #endif 
       if(aRxBuffer[0]==0x00)//if(Key_AccessTimes(&key1,KEY_ACCESS_READ)!=0)//按键被按下过
      {
        printf("=================================================\r\n");
        printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
        vTaskList((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
       
        printf("\r\n任务名       运行计数         使用率\r\n");
        vTaskGetRunTimeStats((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
        Key_AccessTimes(&key1,KEY_ACCESS_WRITE_CLEAR);
      }
      
      if(aRxBuffer[0]==0xa1)//if(Key_AccessTimes(&key2,KEY_ACCESS_READ)!=0)//按键被按下过
      {         
         // printf("KEY2按下，启动单次定时器中断，50ms后在定时器中断给任务vTaskMsgPro发送消息\r\n");
		  aRxBuffer[0]=0;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          
           g_uiCount=aRxBuffer[7];
		  	  
		   /* 向消息队列发数据 */
		   xQueueSendFromISR(xQueue1,
					  (void *)&g_uiCount,
					   &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			Key_AccessTimes(&key2,KEY_ACCESS_WRITE_CLEAR);
		}  
	   if(aRxBuffer[0]==0xa2)
	   {
		  MSG_T   *ptMsg=NULL;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
         /* 初始化结构体指针 */
          ptMsg = &g_tMsg;
          
		  /* 初始化数组 */
		  ptMsg->ucMessageID=aRxBuffer[0];
		  ptMsg->ulData[0]=aRxBuffer[0];
		  ptMsg->usData[1]=aRxBuffer[1];
		  ptMsg->ulData[2]=aRxBuffer[2];
		  ptMsg->usData[3]=aRxBuffer[3];
		  ptMsg->usData[4]=aRxBuffer[4];
		  ptMsg->ulData[5]=aRxBuffer[5];
		  ptMsg->usData[6]=aRxBuffer[6];
		  ptMsg->usData[7]=aRxBuffer[7];
		   
		   /* 向消息队列发数�-�   --WT.EDIT */
			xQueueSendFromISR(xQueue2,
					  (void *)&g_tMsg,
					   &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			Key_AccessTimes(&key2,KEY_ACCESS_WRITE_CLEAR);
	  }
      
      if(aRxBuffer[0]==0xa3)//if(Key_AccessTimes(&key3,KEY_ACCESS_READ)!=0)//按键被按下过
      {         
          MSG_T   *ptMsg=NULL;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
         /* 初始化结构体指针 */
          ptMsg = &g_tMsg;
          
		  /* 初始化数组 */
		  ptMsg->ucMessageID=aRxBuffer[0];
		  ptMsg->ulData[0]=aRxBuffer[1];
		  ptMsg->usData[0]=aRxBuffer[0];
      
         /* 向消息队列发数据 */
         xQueueSendFromISR(xQueue3,
                  (void *)&g_tMsg,
                  &xHigherPriorityTaskWoken);
		 portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
         Key_AccessTimes(&key3,KEY_ACCESS_WRITE_CLEAR);  
				  
	   }
       if(aRxBuffer[0]==0xa4)
	   {		   
		  MSG_T   *ptMsg=NULL;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
         /* 初始化结构体指针 */
          ptMsg = &g_tMsg;
          
		  /* 初始化数组 */
		   
		 
		  #if 0
		  //ptMsg->ucMessageID=aRxBuffer[0];
		//  ptMsg->ulData[0]=aRxBuffer[];
		  ptMsg->usData[0]=aRxBuffer[0];
		  ptMsg->usData[1]=aRxBuffer[1];
		  ptMsg->usData[2]=aRxBuffer[2];
		  ptMsg->usData[3]=aRxBuffer[3];
		  ptMsg->usData[4]=aRxBuffer[4];
		  ptMsg->usData[5]=aRxBuffer[5];
		   
		  printf("usData[0]=%#x\n",ptMsg->usData[0]);
		  printf("usData[1]=%#x\n",ptMsg->usData[1]);
		  printf("usData[2]=%#x\n",ptMsg->usData[2]);
		  printf("usData[3]=%#x\n",ptMsg->usData[3]);
		   printf("usData[4]=%#x\n",ptMsg->usData[4]);
		  printf("usData[5]=%#x\n",ptMsg->usData[5]);
		   #endif
		   /* 向消息队列发数据 */
               xQueueSendFromISR(xQueue4,
                  (void *)&g_tMsg,
                  &xHigherPriorityTaskWoken);
				 

        /* 如果xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        
        Key_AccessTimes(&key3,KEY_ACCESS_WRITE_CLEAR);
	   }
         
     vTaskDelay(20);
  }
		
}

/**
  * 函数功能: LED1任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void vTaskX_axis(void *pvParameters)
{
  BaseType_t xResult;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 设置最大等待时间为300ms */
  uint8_t ucQueueMsgValue;
	

  while(1)
  {
    xResult = xQueueReceive(xQueue1,                   /* 消息队列句柄 */
                            (void *)&ucQueueMsgValue,  		   /* 这里获取的是结构体的地址 */
                            (TickType_t)xMaxBlockTime);/* 设置阻塞时间 */


    if(xResult == pdPASS)
    {
      /* 成功接收，并通过串口将数据打印出来 */
		printf("X_axis:ucQueueMsgValue = %#x\r\n", ucQueueMsgValue);
	 // STEPMOTOR_DisMoveAbs(AXIS_X,400,step_accel,step_decel,set_speed);//X轴移动到400mm的位置 局对距离
	   STEPMOTOR_AxisMoveRel(AXIS_X,30*SPR*CCW,step_accel,step_decel,set_speed); // Z轴反向移动30圈，相对距离
	  
    }
    else
    {
      LED1_TOGGLE;

    }
  }
}

/***********************************************************
  *
  * 函数功能: LED2任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *
***********************************************************/
static void vTaskY_axis(void *pvParameters)
{
    MSG_T *ptMsg;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 设置最大等待时间为200ms */

  while(1)
  {
   xResult = xQueueReceive(xQueue2,                   /* 消息队列句柄 */
                          (void *)&ptMsg,  /* 存储接收到的数据到变量ucQueueMsgValue中 */
                          (TickType_t)xMaxBlockTime);/* 设置阻塞时间 */
  
    if(xResult == pdPASS)
    {
      
       /* 成功接收，并通过串口将数据打印出来 */
	  printf("Y_axis:ptMsg->ucMessageID = %#x\r\n", ptMsg->ucMessageID);
      printf("Y_axis:ptMsg->usData[0] = %#x\r\n", ptMsg->usData[0]);
      printf("Y_axis:tMsg->usData[1] = %#x\r\n", ptMsg->usData[1]);
	  printf("Y_axis:ptMsg->usData[2] = %#x\r\n", ptMsg->usData[2]);
      printf("Y_axis:tMsg->usData[3] = %#x\r\n", ptMsg->usData[3]);
	  printf("Y_axis:ptMsg->usData[4] = %#x\r\n", ptMsg->usData[4]);
      printf("Y_axis:tMsg->usData[5] = %#x\r\n", ptMsg->usData[5]);
	  STEPMOTOR_AxisMoveRel(AXIS_Y,30*SPR*CCW,step_accel,step_decel,set_speed); // X轴反向移动30圈
		
	 
    }
    else
    {
      LED2_TOGGLE;
    }
	
  }
}

/**************************************************
  *
  * 函数功能: Z_axis任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *
***************************************************/
static void vTaskZ_axis(void *pvParameters)
{
    MSG_T *ptMsg;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 设置最大等待时间为200ms */
	

  while(1)
  {
    xResult = xQueueReceive(xQueue3,                   /* 消息队列句柄 */
                            (void *)&ptMsg,  		   /* 这里获取的是结构体的地址 */
                            (TickType_t)xMaxBlockTime);/* 设置阻塞时间 */


    if(xResult == pdPASS)
    {
      /* 成功接收，并通过串口将数据打印出来 */
	    printf("Z_axis:tMsg->ucMessageID = %#x\r\n", ptMsg->ucMessageID);
		printf("Z_axis:ptMsg->ulData[0] = %#x\r\n", ptMsg->ulData[0]);
		printf("Z_axis:tMsg->usData[0] = %#x\r\n", ptMsg->usData[0]);
	  //STEPMOTOR_DisMoveAbs(AXIS_Z,400,step_accel,step_decel,set_speed);//X轴移动到400mm的位置
	   STEPMOTOR_AxisMoveRel(AXIS_Z,30*SPR*CCW,step_accel,step_decel,set_speed); // Z轴反向移动30圈
	}
    else
    {
      LED3_TOGGLE;

    }
	
  }
}
/**************************************************
  *
  * 函数功能: R_axis任务
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *
***************************************************/
static void vTaskR_axis(void *pvParameters)
{
    int R_axis=0;
	while(1)
    {
       MSG_T *ptMsg;
	   BaseType_t xResult;
	   const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500); /* 设置最大等待时间为200ms */

	  while(1)
	  {
	   xResult = xQueueReceive(xQueue4,                   /* 消息队列句柄 */
							  (void *)&ptMsg,  /* 存储接收到的数据到变量ucQueueMsgValue中 */
							  (TickType_t)xMaxBlockTime);/* 设置阻塞时间 */
	  
		if(xResult == pdPASS)
		{
		  /* 成功接收，并通过串口将数据打印出来 */
			printf("R_axis:ptMsg->ucMessageID = %#x\r\n", ptMsg->ucMessageID);
            R_axis=1;
			  printf("Y_axis:ptMsg->usData[0] = %#x\r\n", ptMsg->usData[0]);
			  printf("Y_axis:tMsg->usData[1] = %#x\r\n", ptMsg->usData[1]);
			  printf("Y_axis:ptMsg->usData[2] = %#x\r\n", ptMsg->usData[2]);
			  printf("Y_axis:tMsg->usData[3] = %#x\r\n", ptMsg->usData[3]);
			  printf("Y_axis:ptMsg->usData[4] = %#x\r\n", ptMsg->usData[4]);
			  printf("Y_axis:tMsg->usData[5] = %#x\r\n", ptMsg->usData[5]);
			
	
		}
		if(R_axis==1)
		{
		   R_axis=0;
		   printf("aRxBuffer[0]=%#x \n",aRxBuffer[0]);
		   printf("aRxBuffer[1]=%#x \n",aRxBuffer[1]);
		   printf("aRxBuffer[2]=%#x \n",aRxBuffer[2]);
		   printf("aRxBuffer[0]=%#x \n",aRxBuffer[3]);
		   printf("aRxBuffer[1]=%#x \n",aRxBuffer[4]);
		   printf("aRxBuffer[2]=%#x \n",aRxBuffer[5]);
		   printf("aRxBuffer[2]=%#x \n",aRxBuffer[6]);
		   printf("aRxBuffer[7]=%#x \n",aRxBuffer[7]);
			STEPMOTOR_AxisMoveRel(AXIS_R,30*SPR*CW,step_accel,step_decel,set_speed);  // R轴正向移动30圈
		
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
  * 函数功能: 创建任务应用
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *
**************************************************/
static void AppTaskCreate (void)
{

    xTaskCreate( vTaskTaskUserIF,   	/* 任务函数  */
                 "vTaskUserIF",     	/* 任务名    */
                 1024,               	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 1,                 	/* 任务优先级*/
                 &xHandleTaskUserIF );  /* 任务句柄  */
	
    xTaskCreate( vTaskX_axis,   	      /* 任务函数  */
                 "vTaskX_axis",     	  /* 任务名    */
                 1024,               	/* 任务栈大小，单位word，也就是4字节 */
                 NULL,              	/* 任务参数  */
                 2,                 	/* 任务优先级*/
                 &xHandleTaskX_axis );  /* 任务句柄  */
	
	
	xTaskCreate( vTaskY_axis,    		      /* 任务函数  */
                 "vTaskY_axis",  		    /* 任务名    */
                 1024,         		    /* 任务栈大小，单位word，也就是4字节 */
                 NULL,        		    /* 任务参数  */
                 3,           		    /* 任务优先级*/
                 &xHandleTaskY_axis);  /* 任务句柄  */
	
	xTaskCreate( vTaskZ_axis,     		    /* 任务函数  */
                 "vTaskZ_axis",   		  /* 任务名    */
                 1024,             		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		  /* 任务参数  */
                 4,               		/* 任务优先级*/
                 &xHandleTaskZ_axis );  /* 任务句柄  */
	
	xTaskCreate( vTaskR_axis,     		    /* 任务函数  */
                 "vTaskR_axis",   		  /* 任务名    */
                 1024,             		/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		  /* 任务参数  */
                 5,               		/* 任务优先级*/
                 &xHandleTaskR_axis );  /* 任务句柄  */
	
}

/***************************************************
 *
 *函数名称：AppObjCreate(void)
 *函数功能：创建消息队列。
 *
 *
****************************************************/
static void AppObjCreate (void)
{
	/* 创建10个uint8_t型消息队列 */
	xQueue1 = xQueueCreate(10, sizeof(uint8_t));
    if( xQueue1 == 0 )
    {
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
		printf("xQueue1 don't set up ERROR !\n");
    }
	
	/* 创建10个存储指针变量的消息队列，由于CM3/CM4内核是32位机，一个指针变量占用4个字节 */
	xQueue2 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue2 == 0 )
    {
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
		printf("xQueue2 don't set up ERROR !\n");
    }
	/* 创建10个存储指针变量的消息队列，由于CM3/CM4内核是32位机，一个指针变量占用4个字节 */
	xQueue3 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue3 == 0 )
    {
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
		printf("xQueue3 don't set up ERROR !\n");
    }
	xQueue4 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue4 == 0 )
    {
        printf("xQueue4 don't set up ERROR !\n");
    }
}



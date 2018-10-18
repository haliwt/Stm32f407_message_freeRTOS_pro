#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "led/bsp_led.h"
//#include "R_axis/r_axis.h" 
#include "usart/bsp_debug_usart.h"
#include "key/bsp_key.h"
//#include "Z_axis/z_axis.h"
#include "StepMotor/bsp_StepMotor.h"

/* Ë½ÓĞÀàĞÍ¶¨Òå --------------------------------------------------------------*/
/* Ë½ÓĞºê¶¨Òå ----------------------------------------------------------------*/
/* Ë½ÓĞ±äÁ¿ ------------------------------------------------------------------*/
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
// ËÙ¶È×î´óÖµÓÉÇı¶¯Æ÷ºÍµç»ú¾ö¶¨£¬ÓĞĞ©×î´óÊÇ1800£¬ÓĞĞ©¿ÉÒÔ´ïµ½4000
__IO uint32_t set_speed  = 3000;         // ËÙ¶È µ¥Î»Îª0.05rad/sec
// ¼ÓËÙ¶ÈºÍ¼õËÙ¶ÈÑ¡È¡Ò»°ã¸ù¾İÊµ¼ÊĞèÒª£¬ÖµÔ½´óËÙ¶È±ä»¯Ô½¿ì£¬¼Ó¼õËÙ½×¶Î±È½Ï¶¶¶¯
// ËùÒÔ¼ÓËÙ¶ÈºÍ¼õËÙ¶ÈÖµÒ»°ãÊÇÔÚÊµ¼ÊÓ¦ÓÃÖĞ¶à³¢ÊÔ³öÀ´µÄ½á¹û
__IO uint32_t step_accel = 100;         // ¼ÓËÙ¶È µ¥Î»Îª0.0.5rad/sec^2
__IO uint32_t step_decel = 100;         // ¼õËÙ¶È µ¥Î»Îª0.05rad/sec^2

typedef struct Msg
{
	uint8_t  ucMessageID;
	uint8_t usData[8];
	uint8_t ulData[8];
}MSG_T;

MSG_T  g_tMsg; /* ¶¨ÒåÒ»¸ö½á¹¹ÌåÓÃÓÚÏûÏ¢¶ÓÁĞ */

/* À©Õ¹±äÁ¿ ------------------------------------------------------------------*/


/* Ë½ÓĞº¯ÊıÔ­ĞÎ --------------------------------------------------------------*/
static void vTaskTaskUserIF(void *pvParameters);
static void vTaskX_axis(void *pvParameters);
static void vTaskY_axis(void *pvParameters);
static void vTaskZ_axis(void *pvParameters);
static void vTaskR_axis(void *pvParameters);  //WT.EDIT 
static void AppTaskCreate (void);
static void AppObjCreate (void);

/* º¯ÊıÌå --------------------------------------------------------------------*/
/**
  * º¯Êı¹¦ÄÜ: ÏµÍ³Ê±ÖÓÅäÖÃ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //Ê¹ÄÜPWRÊ±ÖÓ

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //ÉèÖÃµ÷Ñ¹Æ÷Êä³öµçÑ¹¼¶±ğ1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // Íâ²¿¾§Õñ£¬8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //´ò¿ªHSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //´ò¿ªPLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLÊ±ÖÓÔ´Ñ¡ÔñHSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8·ÖÆµMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336±¶Æµ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2·ÖÆµ£¬µÃµ½168MHzÖ÷Ê±ÖÓ
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/Ëæ»úÊı²úÉúÆ÷µÈµÄÖ÷PLL·ÖÆµÏµÊı
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ÏµÍ³Ê±ÖÓ£º168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBÊ±ÖÓ£º 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1Ê±ÖÓ£º42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2Ê±ÖÓ£º84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // Ê¹ÄÜCSS¹¦ÄÜ£¬ÓÅÏÈÊ¹ÓÃÍâ²¿¾§Õñ£¬ÄÚ²¿Ê±ÖÓÔ´Îª±¸ÓÃ
  
 	// HAL_RCC_GetHCLKFreq()/1000    1msÖĞ¶ÏÒ»´Î
	// HAL_RCC_GetHCLKFreq()/100000	 10usÖĞ¶ÏÒ»´Î
	// HAL_RCC_GetHCLKFreq()/1000000 1usÖĞ¶ÏÒ»´Î
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ÅäÖÃ²¢Æô¶¯ÏµÍ³µÎ´ğ¶¨Ê±Æ÷
  /* ÏµÍ³µÎ´ğ¶¨Ê±Æ÷Ê±ÖÓÔ´ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ÏµÍ³µÎ´ğ¶¨Ê±Æ÷ÖĞ¶ÏÓÅÏÈ¼¶ÅäÖÃ */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * º¯Êı¹¦ÄÜ: Ö÷º¯Êı.
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  */
int main(void)
{
  
  /* ¸´Î»ËùÓĞÍâÉè£¬³õÊ¼»¯Flash½Ó¿ÚºÍÏµÍ³µÎ´ğ¶¨Ê±Æ÷ */
  HAL_Init();
  /* ÅäÖÃÏµÍ³Ê±ÖÓ */
  SystemClock_Config();
  
  MX_DEBUG_USART_Init();

   /* ³õÊ¼»¯LED */
  LED_GPIO_Init();
  /* °å×Ó°´¼ü³õÊ¼»¯ */
  KEY_GPIO_Init();
  /* »ù±¾¶¨Ê±Æ÷³õÊ¼»¯£º100usÖĞ¶ÏÒ»´Î */
	
	 STEPMOTOR_TIMx_Init();
	/* ´´½¨ÈÎÎñ */
	AppTaskCreate();
  /* ´´½¨ÈÎÎñÍ¨ĞÅ»úÖÆ */
	AppObjCreate();	
  /* Æô¶¯µ÷¶È£¬¿ªÊ¼Ö´ĞĞÈÎÎñ */
  vTaskStartScheduler();
  
  /* ÎŞÏŞÑ­»· */
  while (1)
  {
  }
}

/**
  * º¯Êı¹¦ÄÜ: ½Ó¿ÚÏûÏ¢´¦Àí
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  */
static uint32_t g_uiCount = 0; /* ÉèÖÃÎªÈ«¾Ö¾²Ì¬±äÁ¿£¬·½±ãÊı¾İ¸üĞÂ */

static void vTaskTaskUserIF(void *pvParameters)
{
    uint8_t pcWriteBuffer[500];
	
    /* ´´½¨°´¼ü */
    KeyCreate(&key1,GetPinStateOfKey1);
	KeyCreate(&key2,GetPinStateOfKey2);
	KeyCreate(&key3,GetPinStateOfKey3);
 
  printf("KEY1¡¢KEY2ºÍKEY3¶ÔÓ¦²»Í¬ÈÎÎñÇé¿ö\n");

    while(1)
    {
      Key_RefreshState(&key1);//Ë¢ĞÂ°´¼ü×´Ì¬
      Key_RefreshState(&key2);//Ë¢ĞÂ°´¼ü×´Ì¬
      Key_RefreshState(&key3);//Ë¢ĞÂ°´¼ü×´Ì¬
   
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
       if(aRxBuffer[0]==0x00)//if(Key_AccessTimes(&key1,KEY_ACCESS_READ)!=0)//°´¼ü±»°´ÏÂ¹ı
      {
        printf("=================================================\r\n");
        printf("ÈÎÎñÃû      ÈÎÎñ×´Ì¬ ÓÅÏÈ¼¶   Ê£ÓàÕ» ÈÎÎñĞòºÅ\r\n");
        vTaskList((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
       
        printf("\r\nÈÎÎñÃû       ÔËĞĞ¼ÆÊı         Ê¹ÓÃÂÊ\r\n");
        vTaskGetRunTimeStats((char *)&pcWriteBuffer);
        printf("%s\r\n", pcWriteBuffer);
        Key_AccessTimes(&key1,KEY_ACCESS_WRITE_CLEAR);
      }
      
      if(aRxBuffer[0]==0xa1)//if(Key_AccessTimes(&key2,KEY_ACCESS_READ)!=0)//°´¼ü±»°´ÏÂ¹ı
      {         
         // printf("KEY2°´ÏÂ£¬Æô¶¯µ¥´Î¶¨Ê±Æ÷ÖĞ¶Ï£¬50msºóÔÚ¶¨Ê±Æ÷ÖĞ¶Ï¸øÈÎÎñvTaskMsgPro·¢ËÍÏûÏ¢\r\n");
		  aRxBuffer[0]=0;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          
           g_uiCount=aRxBuffer[7];
		  	  
		   /* ÏòÏûÏ¢¶ÓÁĞ·¢Êı¾İ */
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
      
         /* ³õÊ¼»¯½á¹¹ÌåÖ¸Õë */
          ptMsg = &g_tMsg;
          
		  /* ³õÊ¼»¯Êı×é */
		  ptMsg->ucMessageID=aRxBuffer[0];
		  ptMsg->ulData[0]=aRxBuffer[0];
		  ptMsg->usData[1]=aRxBuffer[1];
		  ptMsg->ulData[2]=aRxBuffer[2];
		  ptMsg->usData[3]=aRxBuffer[3];
		  ptMsg->usData[4]=aRxBuffer[4];
		  ptMsg->ulData[5]=aRxBuffer[5];
		  ptMsg->usData[6]=aRxBuffer[6];
		  ptMsg->usData[7]=aRxBuffer[7];
		   
		   /* ÏòÏûÏ¢¶ÓÁĞ·¢Êı¾-İ   --WT.EDIT */
			xQueueSendFromISR(xQueue2,
					  (void *)&g_tMsg,
					   &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			Key_AccessTimes(&key2,KEY_ACCESS_WRITE_CLEAR);
	  }
      
      if(aRxBuffer[0]==0xa3)//if(Key_AccessTimes(&key3,KEY_ACCESS_READ)!=0)//°´¼ü±»°´ÏÂ¹ı
      {         
          MSG_T   *ptMsg=NULL;
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
         /* ³õÊ¼»¯½á¹¹ÌåÖ¸Õë */
          ptMsg = &g_tMsg;
          
		  /* ³õÊ¼»¯Êı×é */
		  ptMsg->ucMessageID=aRxBuffer[0];
		  ptMsg->ulData[0]=aRxBuffer[1];
		  ptMsg->usData[0]=aRxBuffer[0];
      
         /* ÏòÏûÏ¢¶ÓÁĞ·¢Êı¾İ */
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
      
         /* ³õÊ¼»¯½á¹¹ÌåÖ¸Õë */
          ptMsg = &g_tMsg;
          
		  /* ³õÊ¼»¯Êı×é */
		   
		 
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
		   /* ÏòÏûÏ¢¶ÓÁĞ·¢Êı¾İ */
               xQueueSendFromISR(xQueue4,
                  (void *)&g_tMsg,
                  &xHigherPriorityTaskWoken);
				 

        /* Èç¹ûxHigherPriorityTaskWoken = pdTRUE£¬ÄÇÃ´ÍË³öÖĞ¶ÏºóÇĞµ½µ±Ç°×î¸ßÓÅÏÈ¼¶ÈÎÎñÖ´ĞĞ */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        
        Key_AccessTimes(&key3,KEY_ACCESS_WRITE_CLEAR);
	   }
         
     vTaskDelay(20);
  }
		
}

/**
  * º¯Êı¹¦ÄÜ: LED1ÈÎÎñ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  */
static void vTaskX_axis(void *pvParameters)
{
  BaseType_t xResult;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* ÉèÖÃ×î´óµÈ´ıÊ±¼äÎª300ms */
  uint8_t ucQueueMsgValue;
	

  while(1)
  {
    xResult = xQueueReceive(xQueue1,                   /* ÏûÏ¢¶ÓÁĞ¾ä±ú */
                            (void *)&ucQueueMsgValue,  		   /* ÕâÀï»ñÈ¡µÄÊÇ½á¹¹ÌåµÄµØÖ· */
                            (TickType_t)xMaxBlockTime);/* ÉèÖÃ×èÈûÊ±¼ä */


    if(xResult == pdPASS)
    {
      /* ³É¹¦½ÓÊÕ£¬²¢Í¨¹ı´®¿Ú½«Êı¾İ´òÓ¡³öÀ´ */
		printf("X_axis:ucQueueMsgValue = %#x\r\n", ucQueueMsgValue);
	 // STEPMOTOR_DisMoveAbs(AXIS_X,400,step_accel,step_decel,set_speed);//XÖáÒÆ¶¯µ½400mmµÄÎ»ÖÃ ¾Ö¶Ô¾àÀë
	   STEPMOTOR_AxisMoveRel(AXIS_X,30*SPR*CCW,step_accel,step_decel,set_speed); // ZÖá·´ÏòÒÆ¶¯30È¦£¬Ïà¶Ô¾àÀë
	  
    }
    else
    {
      LED1_TOGGLE;

    }
  }
}

/***********************************************************
  *
  * º¯Êı¹¦ÄÜ: LED2ÈÎÎñ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  *
***********************************************************/
static void vTaskY_axis(void *pvParameters)
{
    MSG_T *ptMsg;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* ÉèÖÃ×î´óµÈ´ıÊ±¼äÎª200ms */

  while(1)
  {
   xResult = xQueueReceive(xQueue2,                   /* ÏûÏ¢¶ÓÁĞ¾ä±ú */
                          (void *)&ptMsg,  /* ´æ´¢½ÓÊÕµ½µÄÊı¾İµ½±äÁ¿ucQueueMsgValueÖĞ */
                          (TickType_t)xMaxBlockTime);/* ÉèÖÃ×èÈûÊ±¼ä */
  
    if(xResult == pdPASS)
    {
      
       /* ³É¹¦½ÓÊÕ£¬²¢Í¨¹ı´®¿Ú½«Êı¾İ´òÓ¡³öÀ´ */
	  printf("Y_axis:ptMsg->ucMessageID = %#x\r\n", ptMsg->ucMessageID);
      printf("Y_axis:ptMsg->usData[0] = %#x\r\n", ptMsg->usData[0]);
      printf("Y_axis:tMsg->usData[1] = %#x\r\n", ptMsg->usData[1]);
	  printf("Y_axis:ptMsg->usData[2] = %#x\r\n", ptMsg->usData[2]);
      printf("Y_axis:tMsg->usData[3] = %#x\r\n", ptMsg->usData[3]);
	  printf("Y_axis:ptMsg->usData[4] = %#x\r\n", ptMsg->usData[4]);
      printf("Y_axis:tMsg->usData[5] = %#x\r\n", ptMsg->usData[5]);
	  STEPMOTOR_AxisMoveRel(AXIS_Y,30*SPR*CCW,step_accel,step_decel,set_speed); // XÖá·´ÏòÒÆ¶¯30È¦
		
	 
    }
    else
    {
      LED2_TOGGLE;
    }
	
  }
}

/**************************************************
  *
  * º¯Êı¹¦ÄÜ: Z_axisÈÎÎñ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  *
***************************************************/
static void vTaskZ_axis(void *pvParameters)
{
    MSG_T *ptMsg;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* ÉèÖÃ×î´óµÈ´ıÊ±¼äÎª200ms */
	

  while(1)
  {
    xResult = xQueueReceive(xQueue3,                   /* ÏûÏ¢¶ÓÁĞ¾ä±ú */
                            (void *)&ptMsg,  		   /* ÕâÀï»ñÈ¡µÄÊÇ½á¹¹ÌåµÄµØÖ· */
                            (TickType_t)xMaxBlockTime);/* ÉèÖÃ×èÈûÊ±¼ä */


    if(xResult == pdPASS)
    {
      /* ³É¹¦½ÓÊÕ£¬²¢Í¨¹ı´®¿Ú½«Êı¾İ´òÓ¡³öÀ´ */
	    printf("Z_axis:tMsg->ucMessageID = %#x\r\n", ptMsg->ucMessageID);
		printf("Z_axis:ptMsg->ulData[0] = %#x\r\n", ptMsg->ulData[0]);
		printf("Z_axis:tMsg->usData[0] = %#x\r\n", ptMsg->usData[0]);
	  //STEPMOTOR_DisMoveAbs(AXIS_Z,400,step_accel,step_decel,set_speed);//XÖáÒÆ¶¯µ½400mmµÄÎ»ÖÃ
	   STEPMOTOR_AxisMoveRel(AXIS_Z,30*SPR*CCW,step_accel,step_decel,set_speed); // ZÖá·´ÏòÒÆ¶¯30È¦
	}
    else
    {
      LED3_TOGGLE;

    }
	
  }
}
/**************************************************
  *
  * º¯Êı¹¦ÄÜ: R_axisÈÎÎñ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  *
***************************************************/
static void vTaskR_axis(void *pvParameters)
{
    int R_axis=0;
	while(1)
    {
       MSG_T *ptMsg;
	   BaseType_t xResult;
	   const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500); /* ÉèÖÃ×î´óµÈ´ıÊ±¼äÎª200ms */

	  while(1)
	  {
	   xResult = xQueueReceive(xQueue4,                   /* ÏûÏ¢¶ÓÁĞ¾ä±ú */
							  (void *)&ptMsg,  /* ´æ´¢½ÓÊÕµ½µÄÊı¾İµ½±äÁ¿ucQueueMsgValueÖĞ */
							  (TickType_t)xMaxBlockTime);/* ÉèÖÃ×èÈûÊ±¼ä */
	  
		if(xResult == pdPASS)
		{
		  /* ³É¹¦½ÓÊÕ£¬²¢Í¨¹ı´®¿Ú½«Êı¾İ´òÓ¡³öÀ´ */
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
			STEPMOTOR_AxisMoveRel(AXIS_R,30*SPR*CW,step_accel,step_decel,set_speed);  // RÖáÕıÏòÒÆ¶¯30È¦
		
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
  * º¯Êı¹¦ÄÜ: ´´½¨ÈÎÎñÓ¦ÓÃ
  * ÊäÈë²ÎÊı: ÎŞ
  * ·µ »Ø Öµ: ÎŞ
  * Ëµ    Ã÷: ÎŞ
  *
**************************************************/
static void AppTaskCreate (void)
{

    xTaskCreate( vTaskTaskUserIF,   	/* ÈÎÎñº¯Êı  */
                 "vTaskUserIF",     	/* ÈÎÎñÃû    */
                 1024,               	/* ÈÎÎñÕ»´óĞ¡£¬µ¥Î»word£¬Ò²¾ÍÊÇ4×Ö½Ú */
                 NULL,              	/* ÈÎÎñ²ÎÊı  */
                 1,                 	/* ÈÎÎñÓÅÏÈ¼¶*/
                 &xHandleTaskUserIF );  /* ÈÎÎñ¾ä±ú  */
	
    xTaskCreate( vTaskX_axis,   	      /* ÈÎÎñº¯Êı  */
                 "vTaskX_axis",     	  /* ÈÎÎñÃû    */
                 1024,               	/* ÈÎÎñÕ»´óĞ¡£¬µ¥Î»word£¬Ò²¾ÍÊÇ4×Ö½Ú */
                 NULL,              	/* ÈÎÎñ²ÎÊı  */
                 2,                 	/* ÈÎÎñÓÅÏÈ¼¶*/
                 &xHandleTaskX_axis );  /* ÈÎÎñ¾ä±ú  */
	
	
	xTaskCreate( vTaskY_axis,    		      /* ÈÎÎñº¯Êı  */
                 "vTaskY_axis",  		    /* ÈÎÎñÃû    */
                 1024,         		    /* ÈÎÎñÕ»´óĞ¡£¬µ¥Î»word£¬Ò²¾ÍÊÇ4×Ö½Ú */
                 NULL,        		    /* ÈÎÎñ²ÎÊı  */
                 3,           		    /* ÈÎÎñÓÅÏÈ¼¶*/
                 &xHandleTaskY_axis);  /* ÈÎÎñ¾ä±ú  */
	
	xTaskCreate( vTaskZ_axis,     		    /* ÈÎÎñº¯Êı  */
                 "vTaskZ_axis",   		  /* ÈÎÎñÃû    */
                 1024,             		/* ÈÎÎñÕ»´óĞ¡£¬µ¥Î»word£¬Ò²¾ÍÊÇ4×Ö½Ú */
                 NULL,           		  /* ÈÎÎñ²ÎÊı  */
                 4,               		/* ÈÎÎñÓÅÏÈ¼¶*/
                 &xHandleTaskZ_axis );  /* ÈÎÎñ¾ä±ú  */
	
	xTaskCreate( vTaskR_axis,     		    /* ÈÎÎñº¯Êı  */
                 "vTaskR_axis",   		  /* ÈÎÎñÃû    */
                 1024,             		/* ÈÎÎñÕ»´óĞ¡£¬µ¥Î»word£¬Ò²¾ÍÊÇ4×Ö½Ú */
                 NULL,           		  /* ÈÎÎñ²ÎÊı  */
                 5,               		/* ÈÎÎñÓÅÏÈ¼¶*/
                 &xHandleTaskR_axis );  /* ÈÎÎñ¾ä±ú  */
	
}

/***************************************************
 *
 *º¯ÊıÃû³Æ£ºAppObjCreate(void)
 *º¯Êı¹¦ÄÜ£º´´½¨ÏûÏ¢¶ÓÁĞ¡£
 *
 *
****************************************************/
static void AppObjCreate (void)
{
	/* ´´½¨10¸öuint8_tĞÍÏûÏ¢¶ÓÁĞ */
	xQueue1 = xQueueCreate(10, sizeof(uint8_t));
    if( xQueue1 == 0 )
    {
        /* Ã»ÓĞ´´½¨³É¹¦£¬ÓÃ»§¿ÉÒÔÔÚÕâÀï¼ÓÈë´´½¨Ê§°ÜµÄ´¦Àí»úÖÆ */
		printf("xQueue1 don't set up ERROR !\n");
    }
	
	/* ´´½¨10¸ö´æ´¢Ö¸Õë±äÁ¿µÄÏûÏ¢¶ÓÁĞ£¬ÓÉÓÚCM3/CM4ÄÚºËÊÇ32Î»»ú£¬Ò»¸öÖ¸Õë±äÁ¿Õ¼ÓÃ4¸ö×Ö½Ú */
	xQueue2 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue2 == 0 )
    {
        /* Ã»ÓĞ´´½¨³É¹¦£¬ÓÃ»§¿ÉÒÔÔÚÕâÀï¼ÓÈë´´½¨Ê§°ÜµÄ´¦Àí»úÖÆ */
		printf("xQueue2 don't set up ERROR !\n");
    }
	/* ´´½¨10¸ö´æ´¢Ö¸Õë±äÁ¿µÄÏûÏ¢¶ÓÁĞ£¬ÓÉÓÚCM3/CM4ÄÚºËÊÇ32Î»»ú£¬Ò»¸öÖ¸Õë±äÁ¿Õ¼ÓÃ4¸ö×Ö½Ú */
	xQueue3 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue3 == 0 )
    {
        /* Ã»ÓĞ´´½¨³É¹¦£¬ÓÃ»§¿ÉÒÔÔÚÕâÀï¼ÓÈë´´½¨Ê§°ÜµÄ´¦Àí»úÖÆ */
		printf("xQueue3 don't set up ERROR !\n");
    }
	xQueue4 = xQueueCreate(10, sizeof(struct Msg *));
    if( xQueue4 == 0 )
    {
        printf("xQueue4 don't set up ERROR !\n");
    }
}



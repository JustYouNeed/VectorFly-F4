/**
  *******************************************************************************************************
  * File Name: app_led.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-7-4
  * Brief: LED应用层
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-7-4
	*			Mod: 建立本文件
  *
  *******************************************************************************************************
  */	
/*
    *******************************************************************************************************
    *                              INCLUDE FILES
    *******************************************************************************************************
  */	
# include "app.h"

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
TaskHandle_t ledTaskHandle = NULL;
void task_LEDTask(void *p_arg);


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void task_LEDTask(void *p_arg)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = 5;
	
	/*  先获取一次系统时间,因为调用的是绝对延迟  */
	xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		bsp_led_Thread();
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}
/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void led_CreateLEDTask(void)
{
	bsp_led_StartFlash(LED_A, 2, 40, 1000);
	taskENTER_CRITICAL();
	xTaskCreate(task_LEDTask, "LED Task", LED_TASK_STK_SIZE, NULL, LED_TASK_PRIO, &ledTaskHandle);
	taskEXIT_CRITICAL();
}

/********************************************  END OF FILE  *******************************************/


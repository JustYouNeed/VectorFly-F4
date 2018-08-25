/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "app.h"
/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
# define START_TASK_PRIO		2
# define START_TASK_STK_SIZE		256
TaskHandle_t startTaskHandle = NULL;
void task_StartTask(void *p_arg);


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
int main(void)
{ 
	bsp_Config();
	ano_Init();
	usblink_Init();
	uartlink_Init();
	xTaskCreate(task_StartTask, "Start Task", START_TASK_STK_SIZE, NULL, START_TASK_PRIO, &startTaskHandle);
	vTaskStartScheduler();
	while(1);
}

void test(void *p_arg)
{
	anoTxPacket_t packet;
	sensorData_t sensor;
	while(1)
	{
		sensors_AcquireData(&sensor);
		
		packet.DataLength = 18;
		packet.FunCode = DTH_SENSOR;
		
		packet.Data[0] = ((int)sensor.acc.x * 100 >> 8);
		packet.Data[1] = sensor.acc.x * 100;
		packet.Data[2] = ((int)sensor.acc.y * 100 >> 8);
		packet.Data[3] = sensor.acc.y * 100;
		packet.Data[4] = ((int)sensor.acc.z * 100 >> 8);
		packet.Data[5] = sensor.acc.z * 100;
		
		packet.Data[6] = ((int)sensor.gyro.x * 100 >> 8);
		packet.Data[7] = sensor.gyro.x * 100;
		packet.Data[8] = ((int)sensor.gyro.y * 100 >> 8);
		packet.Data[9] = sensor.gyro.y * 100;
		packet.Data[10] = ((int)sensor.gyro.z * 100 >> 8);
		packet.Data[11] = sensor.gyro.z* 100 ;
		
		packet.Data[12] = ((int)sensor.mag.x * 100 >> 8);
		packet.Data[13] = sensor.mag.x * 100;
		packet.Data[14] = ((int)sensor.mag.y * 100 >> 8);
		packet.Data[15] = sensor.mag.y * 100;
		packet.Data[16] = ((int)sensor.mag.z * 100 >> 8);
		packet.Data[17] = sensor.mag.z* 100 ;
		
		usblink_SendPacket(&packet);
		
		vTaskDelay(50);
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
void task_StartTask(void *p_arg)
{
	taskENTER_CRITICAL();
	bsp_led_StartFlash(LED_ALL, 5, 50, LED_KEEP_FLASH);
	
	xTaskCreate(task_LEDTask, "LED Task", LED_TASK_STK_SIZE, NULL, LED_TASK_PRIO, NULL);
	xTaskCreate(usblink_TxTask, "Start Task", 512, NULL, START_TASK_PRIO + 3, NULL);
	xTaskCreate(usblink_RxTask, "Start Task", 512, NULL, START_TASK_PRIO + 4, NULL);
	
	xTaskCreate(test, "Start Task", 128, NULL, START_TASK_PRIO + 3, NULL);
	
	xTaskCreate(uartlink_RxTask, "uartlink Rx Task", 512, NULL, START_TASK_PRIO + 4, NULL);
	xTaskCreate(uartlink_TxTask, "uartlink Tx Task", 512, NULL, START_TASK_PRIO + 5, NULL);
	xTaskCreate(ano_PacketRxTask, "packet task", 512, NULL, START_TASK_PRIO + 6, NULL);
	
	xTaskCreate(sensors_Task, "sensors Task", 1024, NULL, START_TASK_PRIO + 5, NULL);
	
	vTaskDelete(startTaskHandle);
	taskEXIT_CRITICAL();
}

/********************************************  END OF FILE  *******************************************/



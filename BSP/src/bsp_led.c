/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-2-17
  * Brief: 本文件为板级LED灯驱动
  *******************************************************************************************************
  * History
	*		1.Author:	Vector
	*			Date:	2018-2-17
  *			Mod:	建立文件
	*
	*		2.Author: Vector
	*			Date: 2018-8-2
	*			Mod: 1.将LED封装成一个结构体,便于管理,使用更加统一
	*					 2.新增LED持续闪烁功能
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*  定义按键开关时引脚的状态  */
# define LED_OFF				0
# define LED_ON					1

/*  LED引脚相关定义  */
# define LED_A_PORT		GPIOA
# define LED_A_PIN		GPIO_Pin_2

# define LED_B_PORT		GPIOA
# define LED_B_PIN		GPIO_Pin_8

# define LED_C_PORT		GPIOB
# define LED_C_PIN		GPIO_Pin_4

# define LED_D_PORT		GPIOA
# define LED_D_PIN		GPIO_Pin_1

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static LED_TypeDef LED[LED_COUNT];
static bool isInit = false;

/*
*********************************************************************************************************
*                                   bsp_led_Init       
*
* Description: 初始化LED引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_Init(void)
{
	if(isInit) return ;
	
	uint8_t i = 0;
  GPIO_InitTypeDef  GPIO_InitStructure;
	
# ifdef VECTOR_F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
# else	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
# endif
	
	LED[LED_A].Port = LED_A_PORT;
	LED[LED_A].Pin = LED_A_PIN;
	
	LED[LED_B].Port = LED_B_PORT;
	LED[LED_B].Pin = LED_B_PIN;
	
	LED[LED_C].Port = LED_C_PORT;
	LED[LED_C].Pin = LED_C_PIN;
	
	LED[LED_D].Port = LED_D_PORT;
	LED[LED_D].Pin = LED_D_PIN;
	
	for(i = 0; i < LED_COUNT; i++)
	{		
		LED[i].ucID = i;
		LED[i].ucState = LED_OFF;
		LED[i].usCount = 0;
		LED[i].usCycle = 0;
		LED[i].usCycleCount = 0;
		LED[i].usBrightTime = 0;
		LED[i].usDarkTime = 0;
		LED[i].ucOFF = 0;
		LED[i].eMode = LED_MODE_K;	/*  默认都为高电平点亮  */
		
		bsp_rcc_GPIOClcokCmd(LED[i].Port);
		
		GPIO_InitStructure.GPIO_Pin = LED[i].Pin;
		GPIO_Init(LED[i].Port, &GPIO_InitStructure);
	}
	bsp_led_OFF(LED_ALL);			/*  初始化完默认关闭所有LED  */
	
	isInit = true;
}


/*
*********************************************************************************************************
*                                          bsp_led_ON
*
* Description: 打开一个LED灯
*             
* Arguments  : 1> LEDx:	要打开的LED灯编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_ON(uint8_t LEDx)
{
	if(!isInit && LEDx >= LED_COUNT) return ;
	
	if(LEDx == LED_ALL)
	{
		for(uint8_t i = 0; i < LED_COUNT; i++)
		{
			if(LED[i].eMode == LED_MODE_A)
				PIN_OUT_HIGH(LED[i].Port, LED[i].Pin);
			else
				PIN_OUT_LOW(LED[i].Port, LED[i].Pin);
		}
	}
	else
	{
		if(LED[LEDx].eMode == LED_MODE_A)
			PIN_OUT_HIGH(LED[LEDx].Port, LED[LEDx].Pin);
		else
			PIN_OUT_LOW(LED[LEDx].Port, LED[LEDx].Pin);
	}
}

/*
*********************************************************************************************************
*                                          bsp_led_OFF
*
* Description: 关闭一个LED灯
*             
* Arguments  : 1> LEDx:	要关闭的LED灯编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_OFF(uint8_t LEDx)
{
	if(!isInit && LEDx >= LED_COUNT) return ;
	
	if(LEDx == LED_ALL)
	{
		for(uint8_t i = 0; i < LED_COUNT; i++)
		{
			if(LED[i].eMode == LED_MODE_A)
				PIN_OUT_LOW(LED[i].Port, LED[i].Pin);
			else
				PIN_OUT_HIGH(LED[i].Port, LED[i].Pin);
		}
	}
	else
	{
		if(LED[LEDx].eMode == LED_MODE_A)
			PIN_OUT_LOW(LED[LEDx].Port, LED[LEDx].Pin);
		else
			PIN_OUT_HIGH(LED[LEDx].Port, LED[LEDx].Pin);
	}
}


/*
*********************************************************************************************************
*                                       bsp_led_Toggle   
*
* Description: 切换LED灯的状态
*             
* Arguments  : 1> LEDx:	要切换的LED灯编号
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_Toggle(uint8_t LEDx)
{
	if(!isInit && LEDx >= LED_COUNT) return ;
	
	if(LEDx == LED_ALL)
	{
		for(uint8_t i = 0; i < LED_COUNT; i++)
		{
			PIN_TOGGLE(LED[i].Port, LED[i].Pin);
		}
	}
	else
	{
		PIN_TOGGLE(LED[LEDx].Port, LED[LEDx].Pin);
	}
}

/*
*********************************************************************************************************
*                         bsp_led_StartFlash                 
*
* Description: 让一个LED灯开始闪烁
*             
* Arguments  : 1> LEDx: LED编号, 在bsp_led.h中定义
*							 2> BrightTime: 点亮时长
*							 3> DarkTime: 熄灭时长
*							 4> Cycle: 闪烁次数
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_StartFlash(uint8_t LEDx, uint16_t BrightTime, uint16_t DarkTime, uint16_t Cycle)
{
	uint8_t i = 0;
	
	if(!isInit && LEDx >= LED_COUNT) return ;
	if(BrightTime == 0 || LED[LEDx].ucOFF == 1) return;
	
	if(LEDx == LED_ALL)
	{
		for(i = 0; i < LED_COUNT; i++)
		{
			LED[i].usBrightTime = BrightTime;
			LED[i].usDarkTime = DarkTime;
			LED[i].usCycle = Cycle;
			LED[i].usCount = 0;
			LED[i].ucState = 0;
			LED[i].ucOFF = 0;
			LED[i].usCycleCount = 0;
			LED[i].ucEnable = 1;
		}
	}
	else
	{
		LED[LEDx].usBrightTime = BrightTime;
		LED[LEDx].usDarkTime = DarkTime;
		LED[LEDx].usCycle = Cycle;
		LED[LEDx].usCount = 0;
		LED[LEDx].ucState = 0;
		LED[LEDx].ucOFF = 0;
		LED[LEDx].usCycleCount = 0;
		LED[LEDx].ucEnable = 1;
	}
	bsp_led_ON(LED[LEDx].ucID);
}

/*
*********************************************************************************************************
*                      bsp_led_StopFlash                    
*
* Description: 让一个LED灯停止闪烁
*             
* Arguments  : 1> LEDx: LED编号,在bsp_led.h中定义
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_led_StopFlash(uint8_t LEDx)
{
	if(!isInit && LEDx >= LED_COUNT) return ;
	
	if(LEDx == LED_ALL)
	{
		for(uint8_t i = 0; i < LED_COUNT; i++)
		{
			LED[i].ucOFF = 0;
			LED[i].usBrightTime = 0;
			LED[i].usCount = 0;
			LED[i].usDarkTime = 0;
			LED[i].ucEnable = 0;
			LED[i].usCycle = 0;
			LED[i].usCycleCount = 0;
		}
	}
	else
	{
		LED[LEDx].ucOFF = 0;
		LED[LEDx].usBrightTime = 0;
		LED[LEDx].usCount = 0;
		LED[LEDx].usDarkTime = 0;
		LED[LEDx].ucEnable = 0;
		LED[LEDx].usCycle = 0;
		LED[LEDx].usCycleCount = 0;
	}
	bsp_led_OFF(LED[LEDx].ucID);
}


/*
*********************************************************************************************************
*                        bsp_led_GetFlashState                  
*
* Description: 获取灯的闪烁状态,是否闪烁完成
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
bool bsp_led_GetFlashState(uint8_t LEDx)
{
	return LED[LEDx].usCycleCount >= LED[LEDx].usCycle;
}


/*
*********************************************************************************************************
*                        bsp_led_Thread                  
*
* Description: LED灯的闪烁处理线程,如果需要灯闪烁功能,则应该定周期调用该函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 推荐调用周期为10ms
*********************************************************************************************************
*/
void bsp_led_Thread(void)
{
	uint8_t i = 0;
	
	/*  循环处理每一个LED  */
	for(i = 0; i < LED_COUNT; i++)
	{
		/*  如果关闭时间为0或者灯已经关闭了,就不需要处理这个灯了  */
		if(LED[i].ucEnable == 0 || LED[i].usDarkTime == 0 || LED[i].ucOFF == 1) continue;
		
		/*  如果当前灯的状态是点亮的,则计算还有多长时间关闭  */
		if(LED[i].ucState == 0)
		{
			if(LED[i].usDarkTime > 0)	/*  只有在灯有关闭时间才进行处理  */
			{
				if(++ LED[i].usCount >= LED[i].usBrightTime)		/*  开启时间已经到了,关闭灯  */
				{
					bsp_led_OFF(LED[i].ucID);
					LED[i].usCount = 0;
					LED[i].ucState = 1;
				}
			}
			else	
			{
				;
			}
		}
		else if(LED[i].ucState == 1)		/*  如果当前状态为关闭的,计算还有多长时间点亮  */
		{
			if(++ LED[i].usCount >= LED[i].usDarkTime)		/*  如果关闭时长已经到了，就点亮灯  */
			{
				/*  如果设置了一直闪烁的话就不需要处理  */
				if(LED[i].usCycle > 0 && LED[i].usCycle != LED_KEEP_FLASH)
				{
					if(++ LED[i].usCycleCount >= LED[i].usCycle)
					{
						LED[i].ucEnable = 0;
					}
					
					if(LED[i].ucEnable == 0)
					{
						LED[i].usDarkTime = 0;
						continue;
					}
				}
				
				LED[i].usCount = 0;
				LED[i].ucState = 0;
				
				bsp_led_ON(LED[i].ucID);
			}
		}
	}
}

/********************************************  END OF FILE  *******************************************/

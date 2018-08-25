/**
  *******************************************************************************************************
  * File Name: bsp_led.c
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-2-17
  * Brief: ���ļ�Ϊ�弶LED������
  *******************************************************************************************************
  * History
	*		1.Author:	Vector
	*			Date:	2018-2-17
  *			Mod:	�����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-8-2
	*			Mod: 1.��LED��װ��һ���ṹ��,���ڹ���,ʹ�ø���ͳһ
	*					 2.����LED������˸����
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*  ���尴������ʱ���ŵ�״̬  */
# define LED_OFF				0
# define LED_ON					1

/*  LED������ض���  */
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
* Description: ��ʼ��LED����
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
		LED[i].eMode = LED_MODE_K;	/*  Ĭ�϶�Ϊ�ߵ�ƽ����  */
		
		bsp_rcc_GPIOClcokCmd(LED[i].Port);
		
		GPIO_InitStructure.GPIO_Pin = LED[i].Pin;
		GPIO_Init(LED[i].Port, &GPIO_InitStructure);
	}
	bsp_led_OFF(LED_ALL);			/*  ��ʼ����Ĭ�Ϲر�����LED  */
	
	isInit = true;
}


/*
*********************************************************************************************************
*                                          bsp_led_ON
*
* Description: ��һ��LED��
*             
* Arguments  : 1> LEDx:	Ҫ�򿪵�LED�Ʊ��
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
* Description: �ر�һ��LED��
*             
* Arguments  : 1> LEDx:	Ҫ�رյ�LED�Ʊ��
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
* Description: �л�LED�Ƶ�״̬
*             
* Arguments  : 1> LEDx:	Ҫ�л���LED�Ʊ��
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
* Description: ��һ��LED�ƿ�ʼ��˸
*             
* Arguments  : 1> LEDx: LED���, ��bsp_led.h�ж���
*							 2> BrightTime: ����ʱ��
*							 3> DarkTime: Ϩ��ʱ��
*							 4> Cycle: ��˸����
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
* Description: ��һ��LED��ֹͣ��˸
*             
* Arguments  : 1> LEDx: LED���,��bsp_led.h�ж���
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
* Description: ��ȡ�Ƶ���˸״̬,�Ƿ���˸���
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
* Description: LED�Ƶ���˸�����߳�,�����Ҫ����˸����,��Ӧ�ö����ڵ��øú���
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �Ƽ���������Ϊ10ms
*********************************************************************************************************
*/
void bsp_led_Thread(void)
{
	uint8_t i = 0;
	
	/*  ѭ������ÿһ��LED  */
	for(i = 0; i < LED_COUNT; i++)
	{
		/*  ����ر�ʱ��Ϊ0���ߵ��Ѿ��ر���,�Ͳ���Ҫ�����������  */
		if(LED[i].ucEnable == 0 || LED[i].usDarkTime == 0 || LED[i].ucOFF == 1) continue;
		
		/*  �����ǰ�Ƶ�״̬�ǵ�����,����㻹�ж೤ʱ��ر�  */
		if(LED[i].ucState == 0)
		{
			if(LED[i].usDarkTime > 0)	/*  ֻ���ڵ��йر�ʱ��Ž��д���  */
			{
				if(++ LED[i].usCount >= LED[i].usBrightTime)		/*  ����ʱ���Ѿ�����,�رյ�  */
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
		else if(LED[i].ucState == 1)		/*  �����ǰ״̬Ϊ�رյ�,���㻹�ж೤ʱ�����  */
		{
			if(++ LED[i].usCount >= LED[i].usDarkTime)		/*  ����ر�ʱ���Ѿ����ˣ��͵�����  */
			{
				/*  ���������һֱ��˸�Ļ��Ͳ���Ҫ����  */
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

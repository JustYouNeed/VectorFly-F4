/**
  *******************************************************************************************************
  * File Name: bsp_led.h
  * Author: Vector
  * Version: V1.1.0
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
	*			Mod: ΪLED�ṹ��������������,���ڱ���LED GPIO_PORT, GPIO_Pin,����LED���Žӷ�,��LED�����ͳһ
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_LED_H
# define __BSP_LED_H

# define LED_KEEP_FLASH		9999


/*  LED IDö�ٱ���  */
typedef enum
{
	LED_ALL = 0x0,
	LED_A,
	LED_B,
	LED_C,
	LED_D,
	LED_COUNT,
}LED_EnumTypeDef;

/*  LED�ӷ�ö������,�����ֽӷ�,
		һ��GPIO����ߵ�ƽ����,һ������͵�ƽ����  */
typedef enum
{
	LED_MODE_A = 0x0,		/*  �ߵ�ƽ����  */
	LED_MODE_K,					/*  �͵�ƽ����  */
}ledMode_EnumTypeDef;

typedef struct
{
	GPIO_TypeDef* Port;
	uint16_t			Pin;
	uint8_t 			eMode;
	uint8_t ucID;					/*  LED ID  */
	uint8_t ucOFF;				/*  �ر�LED��  */
	uint8_t ucState;			/*  ״̬  */
	uint16_t usBrightTime;/*  ����ʱ��,�����߳�����  */
	uint16_t usDarkTime;	/*  �ر�ʱ��,�����߳�����  */
	uint16_t usCycle;			/*  ѭ������  */
	uint16_t usCount;			/*  ��������  */
	uint16_t usCycleCount;/*  �Ѿ�ѭ���˶��ٴ�  */
	uint8_t ucEnable;
}LED_TypeDef;


void bsp_led_Init(void);
void bsp_led_Toggle(uint8_t LEDx);
void bsp_led_ON(uint8_t LEDx);
void bsp_led_OFF(uint8_t LEDx);
uint8_t bsp_led_GetState(uint8_t LEDx);
void bsp_led_StartFlash(uint8_t LEDx, uint16_t BrightTime, uint16_t DarkTime, uint16_t Cycle);
void bsp_led_StopFlash(uint8_t LEDx);
bool bsp_led_GetFlashState(uint8_t LEDx);
void bsp_led_Thread(void);
# endif


/********************************************  END OF FILE  *******************************************/

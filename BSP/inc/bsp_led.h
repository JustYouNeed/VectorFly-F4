/**
  *******************************************************************************************************
  * File Name: bsp_led.h
  * Author: Vector
  * Version: V1.1.0
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
	*			Mod: 为LED结构体新增三个变量,用于保存LED GPIO_PORT, GPIO_Pin,还有LED引脚接法,让LED管理更统一
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_LED_H
# define __BSP_LED_H

# define LED_KEEP_FLASH		9999


/*  LED ID枚举变量  */
typedef enum
{
	LED_ALL = 0x0,
	LED_A,
	LED_B,
	LED_C,
	LED_D,
	LED_COUNT,
}LED_EnumTypeDef;

/*  LED接法枚举类型,有两种接法,
		一种GPIO输出高电平点亮,一种输出低电平点亮  */
typedef enum
{
	LED_MODE_A = 0x0,		/*  高电平点亮  */
	LED_MODE_K,					/*  低电平点亮  */
}ledMode_EnumTypeDef;

typedef struct
{
	GPIO_TypeDef* Port;
	uint16_t			Pin;
	uint8_t 			eMode;
	uint8_t ucID;					/*  LED ID  */
	uint8_t ucOFF;				/*  关闭LED灯  */
	uint8_t ucState;			/*  状态  */
	uint16_t usBrightTime;/*  点亮时间,乘以线程周期  */
	uint16_t usDarkTime;	/*  关闭时间,乘以线程周期  */
	uint16_t usCycle;			/*  循环次数  */
	uint16_t usCount;			/*  计数变量  */
	uint16_t usCycleCount;/*  已经循环了多少次  */
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

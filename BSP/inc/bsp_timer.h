/**
  *******************************************************************************************************
  * File Name: bsp_timer.c
  * Author: Vector
  * Version: V1.2.2
  * Date: 2018-2-11
  * Brief: 本文件对滴答定时器进行了一定程度的封装,同时建立了软件定时器,每个定时器可有一个回调函数
  *******************************************************************************************************
  * History
	*		1.Author: Vector
  *			Date: 2018-2-11
  *			Mod: 建立文件
	*		
  *******************************************************************************************************
  */	
# ifndef __BSP_TIMER_H
# define __BSP_TIMER_H

/* 定义软件定时器使用的定时器 */
# define USE_TIM2		1u
# define USE_TIM3   0u
# define USE_TIM4   0u
# define USE_TIM5   0u



# if USE_TIM2 > 0u
	# define BSP_TIMER				TIM2
	# define BSP_TIMER_IRQ		TIM2_IRQn
	# define BSP_TIM_RCC			RCC_APB1Periph_TIM2
#endif

# if USE_TIM3 > 0u
	# define BSP_TIMER				TIM3
	# define BSP_TIMER_IRQ		TIM3_IRQn
	# define BSP_TIM_RCC			RCC_APB1Periph_TIM3
#endif

# if USE_TIM4 > 0u
	# define BSP_TIMER				TIM4
	# define BSP_TIMER_IRQ		TIM4_IRQn
	# define BSP_TIM_RCC			RCC_APB1Periph_TIM4
#endif

# if USE_TIM5 > 0u
	# define BSP_TIMER				TIM5
	# define BSP_TIMER_IRQ		TIM5_IRQn
	# define BSP_TIM_RCC			RCC_APB1Periph_TIM5
#endif


# define SOFT_TIMER_COUNT			8

typedef void (*_cbTimerCallBack)(void);		/*  软件定时器回调函数数据类型  */


/*  软件定时器控制结构体  */
typedef struct	
{
	volatile uint8_t v_ucMode;		/*  模式  */
	volatile uint8_t v_ucFlag;		/*  定时到达标志  */
	volatile uint32_t v_uiCount;	/*  定时计数器  */
	volatile uint32_t v_uiPreLoad;	/*  重装载值  */
	
	uint8_t ucUsed;					/*  是否已经使用  */
	_cbTimerCallBack _cbTimer;	/*  回调函数  */
	
}SoftTimer_TypeDef;


typedef struct
{
	uint8_t ucUsed;							/*  是否已经使用  */
	_cbTimerCallBack _cbTimer;	/*  回调函数  */
}HardTimer_TypeDef;

typedef struct
{
	uint8_t seconds;
	uint8_t mintues;
	uint8_t hours;
	uint8_t days;
	uint8_t week;
	uint8_t month;
	uint16_t years;
	uint64_t runTime;
}SystemTime_Str, *pSystemTime_Str;


/*  软件定时器模式枚举变量  */
typedef enum
{
	TIMER_MODE_ONCE = 0x00,
	TIMER_MODE_AUTO
}TIMER_MODE_ENUM;





void bsp_tim_Init(void);  /* 初始化软件定时器 */
int8_t bsp_tim_HardConfig(uint8_t TIMx, uint32_t uiPeriod);
int8_t bsp_tim_CreateSoftTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer, TIMER_MODE_ENUM eMode);
int8_t bsp_tim_DeleteSoftTimer(uint8_t ucTimerId);


int32_t bsp_tim_GetRunTime(void);
int32_t bsp_TimerSoftCheckRunTime(int32_t iLastTime);
_cbTimerCallBack bsp_tim_SoftGetCB(uint8_t ucTimerId);

void bsp_tim_DelayMs(uint16_t ui_nMs);
void bsp_tim_DelayUs(uint16_t ui_nUs);


void bsp_tim_GetSysTime(SystemTime_Str *Time);


# endif

/********************************************  END OF FILE  *******************************************/



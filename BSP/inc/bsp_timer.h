/**
  *******************************************************************************************************
  * File Name: bsp_timer.c
  * Author: Vector
  * Version: V1.2.2
  * Date: 2018-2-11
  * Brief: ���ļ��Եδ�ʱ��������һ���̶ȵķ�װ,ͬʱ�����������ʱ��,ÿ����ʱ������һ���ص�����
  *******************************************************************************************************
  * History
	*		1.Author: Vector
  *			Date: 2018-2-11
  *			Mod: �����ļ�
	*		
  *******************************************************************************************************
  */	
# ifndef __BSP_TIMER_H
# define __BSP_TIMER_H

/* ���������ʱ��ʹ�õĶ�ʱ�� */
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

typedef void (*_cbTimerCallBack)(void);		/*  �����ʱ���ص�������������  */


/*  �����ʱ�����ƽṹ��  */
typedef struct	
{
	volatile uint8_t v_ucMode;		/*  ģʽ  */
	volatile uint8_t v_ucFlag;		/*  ��ʱ�����־  */
	volatile uint32_t v_uiCount;	/*  ��ʱ������  */
	volatile uint32_t v_uiPreLoad;	/*  ��װ��ֵ  */
	
	uint8_t ucUsed;					/*  �Ƿ��Ѿ�ʹ��  */
	_cbTimerCallBack _cbTimer;	/*  �ص�����  */
	
}SoftTimer_TypeDef;


typedef struct
{
	uint8_t ucUsed;							/*  �Ƿ��Ѿ�ʹ��  */
	_cbTimerCallBack _cbTimer;	/*  �ص�����  */
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


/*  �����ʱ��ģʽö�ٱ���  */
typedef enum
{
	TIMER_MODE_ONCE = 0x00,
	TIMER_MODE_AUTO
}TIMER_MODE_ENUM;





void bsp_tim_Init(void);  /* ��ʼ�������ʱ�� */
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



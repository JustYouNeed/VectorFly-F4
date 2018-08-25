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
	*		2.Author: Vector
	*			Date:	2018-2-17
	*			Mod:	1.�޸���ʱ����־λ����	
	*						2.�޸���ʱ������ʧ������
	*						
	*		3.Author: Vector
	*			Date: 2018-2-27
	*			Mod: 1.�޸ĺ�����bsp_tim_CreateSoftTimerΪbsp_tim_CreateSoftTimer
	*					 2.�޸ĺ�����bsp_tim_DeleteTimerΪbsp_tim_DeleteSoftTimer
	*					 3.�Ż����ֺ���,��ֹ��������Խ��Ŀ���
	*          4.��������bsp_tim_CreateHardTimer����,����Ӳ����ʱ������,Ӳ����ʱ������PIT��ʱ��
	*          5.��������bsp_tim_DeleteHardTimer����,ɾ��Ӳ����ʱ��
	*          6.����HardTimer[]Ӳ����ʱ����������
	*
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# if USE_UCOSIII > 0u
	# include "includes.h"
	
	# ifdef OS_CRITICAL_METHOD
		# define OS_RUNNING		OSRunning
		# define OS_TICKS_SEC	OS_TICKS_PER_SEC
		# define OS_INT_NESTING	OSIntNesting
	# endif
	
	# ifdef	CPU_CFG_CRITICAL_METHOD
		# define OS_RUNNING		OSRunning
		# define OS_TICKS_SEC	OSCfg_TickRate_Hz
		# define OS_INT_NESTING	OSIntNestingCtr
	# endif	
# endif

# if USE_FREERTOS > 0u
	# include "FreeRTOS.h"
	# include "task.h"
	# include "queue.h"
	# include "timers.h"
	extern void xPortSysTickHandler(void);
# endif

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;
__IO int32_t g_iRunTime = 0;


SoftTimer_TypeDef SoftTimer[SOFT_TIMER_COUNT];


/*
*********************************************************************************************************
*                                 bsp_tim_Init         
*
* Description: ��ʼ�������ʱ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_tim_Init(void)
{
	uint8_t i = 0;
# if (USE_UCOSIII > 0u || USE_FREERTOS > 0u)
	uint32_t reload;
# endif
	
	for(i = 0; i < SOFT_TIMER_COUNT; i++)
	{
		SoftTimer[i].v_uiCount = 0;
		SoftTimer[i].v_uiPreLoad = 0;
		SoftTimer[i].v_ucFlag = 0;
		SoftTimer[i].v_ucMode = 0;
		SoftTimer[i]._cbTimer = 0;
	}
	

	
# if (USE_UCOSIII > 0u)		/*  �����Ҫ֧�ֲ���ϵͳ����δ�ʱ����Ҫ�ı�  */
	SysTick_Config(SysTick_CLKSource_HCLK_Div8);
	reload = 168/8;
	reload *= 1000000 / OS_TICKS_SEC;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->LOAD = reload;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
# elif USE_FREERTOS > 0u
	reload = SystemCoreClock / 1000000;
	reload *= 1000000 / configTICK_RATE_HZ;
	
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->LOAD = reload;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
# else
	SysTick_Config(SystemCoreClock / 1000);
# endif
}

/*
*********************************************************************************************************
*                               bsp_tim_SoftDec           
*
* Description: �ݼ�ÿ�������ʱ���ļ�����
*             
* Arguments  : 1> pTimer:SoftTimer_TypeDef�ṹ��ָ�룬ָ��һ�������ʱ��
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ϊ���ļ�˽�к���
*********************************************************************************************************
*/
static void bsp_tim_SoftDec(SoftTimer_TypeDef * pTimer)
{
	if(pTimer->v_uiCount >0)
	{
		if(--pTimer->v_uiCount ==0 )
		{
			pTimer->v_ucFlag = 1;
			
			
			if(pTimer->v_ucMode == TIMER_MODE_AUTO)
			{
				pTimer->v_uiCount = pTimer->v_uiPreLoad;
			}
			
			if(pTimer->_cbTimer)
			{
				pTimer->_cbTimer();
			}
		}
	}
}
/*
*********************************************************************************************************
*                                   SysTick_ISR       
*
* Description: �δ�ʱ���жϷ�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void SysTick_ISR(void)
{
	uint8_t i = 0;
	
	if(s_uiDelayCount > 0)		/*  ��ʱ������  */
	{
		if(-- s_uiDelayCount == 0) s_ucTimeOutFlag = 1;			/*  ��ʱ���  */
	}
	
	g_iRunTime ++;				/*  ����ʱ�������  */
	if(g_iRunTime == 0x7fffffff) g_iRunTime = 0;			/*  ����ʱ�������Ϊ32λ�����ֵΪ 0x7fffffff */

	for(i = 0; i < SOFT_TIMER_COUNT; i++)		/*  �ݼ������ʱ����ֵ  */
	{
		bsp_tim_SoftDec(&SoftTimer[i]);
	}
}
/*
*********************************************************************************************************
*                                SysTick_Handler          
*
* Description: �δ�ʱ���ж�
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void SysTick_Handler(void)
{
# if USE_UCOSIII > 0u
	if(OS_RUNNING == 1)
	{
		OSIntEnter();						//�����ж�
		OSTimeTick();       				//����ucos��ʱ�ӷ������               
		OSIntExit();       	 				//���������л����ж�
	}
# elif USE_FREERTOS > 0u
	if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xPortSysTickHandler();
	}	
# else
	SysTick_ISR();
# endif
}

# if USE_UCOSIII > 0u
/*
*********************************************************************************************************
*                                  bsp_tim_DelayMs        
*
* Description: ���뼶��ʱ����
*             
* Arguments  : 1> ui_nMs:Ҫ��ʱ��ʱ��
*
* Reutrn     : None.
*
* Note(s)    :  ��ʱ���õδ�ʱ��ʵ�֣���Ϊ�δ�ʱ��Ϊ16λ��ʱ�������Ը������ʱʱ��Ϊ65534
*********************************************************************************************************
*/
void bsp_tim_DelayMs(uint16_t ui_nMs)
{
	for(uint16_t i = 0; i < (ui_nMs / 1000); i++)
	{
		bsp_tim_DelayUs(1000);
	}
}
# else
/*
*********************************************************************************************************
*                                  bsp_tim_DelayMs        
*
* Description: ���뼶��ʱ����
*             
* Arguments  : 1> ui_nMs:Ҫ��ʱ��ʱ��
*
* Reutrn     : None.
*
* Note(s)    :  ��ʱ���õδ�ʱ��ʵ�֣���Ϊ�δ�ʱ��Ϊ16λ��ʱ�������Ը������ʱʱ��Ϊ65534
*********************************************************************************************************
*/
void bsp_tim_DelayMs(uint16_t ui_nMs)
{
	if(ui_nMs == 0) return ;
	else if(ui_nMs == 1) ui_nMs = 2;
	
	DISABLE_INT();
	
	s_uiDelayCount = ui_nMs;
	s_ucTimeOutFlag = 0;
	
	ENABLE_INT();
	
	while(1)
	{
		if(s_ucTimeOutFlag == 1) break;
	}
}
# endif

/*
*********************************************************************************************************
*                                 bsp_tim_DelayUs         
*
* Description: ΢�뼶��ʱ����,����ʱ��ժȡ���������ʱ���и���
*             
* Arguments  : 1> ui_nUs:��ʱʱ����΢��
*
* Reutrn     : None.
*
* Note(s)    : �����ʱΪ65535΢��
*********************************************************************************************************
*/
void bsp_tim_DelayUs(uint16_t ui_nUs)
{
		uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
		reload = SysTick->LOAD;                
    ticks = ui_nUs * (SystemCoreClock / 1000000);	 /* ��Ҫ�Ľ����� */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */

    while (1)
    {
			tnow = SysTick->VAL;    
			if (tnow != told)
			{    
				/* SYSTICK��һ���ݼ��ļ����� */    
				if (tnow < told)
				{
						tcnt += told - tnow;    
				}
				/* ����װ�صݼ� */
				else
				{
						tcnt += reload - tnow + told;    
				}        
				told = tnow;

				/* ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
				if (tcnt >= ticks)
				{
					break;
				}
			}  
    }
}
/*
*********************************************************************************************************
*                             bsp_tim_CreateSoftTimer             
*
* Description: ����һ�������ʱ��
*             
* Arguments  : 1> ucTimerId:�����ʱ��ID
*              2> uiPeriod:��ʱ����
*              3> _cbTimer:��ʱ���ص�����
*              4> eMode:��ʱ��ģʽ
*
* Reutrn     : 1> 0: �ɹ�
*              2> ����: �������
*
* Note(s)    : 1.�����ʱ�����������ƣ�������ID�������ʱ������ʱ���ᴴ��ʧ��
*              2.�����ʱ�����ܸ����Ѿ�ʹ�õĶ�ʱ������Ҫ��ԭ����ɾ������ܴ���
*							 3.��ʱ���ص�����ִ��ʱ��Ӧ�����ܶ�,�Ҳ�������ʱ����
*********************************************************************************************************
*/
int8_t bsp_tim_CreateSoftTimer(uint8_t ucTimerId, uint32_t uiPeriod, _cbTimerCallBack  _cbTimer, TIMER_MODE_ENUM eMode)
{
	if(ucTimerId > SOFT_TIMER_COUNT) return -1;				/*  ������ʱ������  */
	if(SoftTimer[ucTimerId].ucUsed == 1) return -2;		/*  �Ѿ�ʹ���˵Ķ�ʱ�������ٴδ���  */
	
	SoftTimer[ucTimerId].v_uiCount = uiPeriod;		/*  ���ö�ʱ����  */
	SoftTimer[ucTimerId].v_ucFlag = 0;		/*  �����ʱ��־λ  */
	SoftTimer[ucTimerId].v_ucMode = eMode;		/*  ��ʱģʽ  */
	SoftTimer[ucTimerId].ucUsed = 1;	/*  �Ѿ�ʹ��  */
	SoftTimer[ucTimerId].v_uiPreLoad = uiPeriod;	/*  �������ֵ  */
	SoftTimer[ucTimerId]._cbTimer = _cbTimer;		/*  ��ʱ���ص�����  */
	
	return 0;
}
/*
*********************************************************************************************************
*                                   bsp_tim_DeleteSoftTimer       
*
* Description: ɾ��һ�������ʱ��
*             
* Arguments  : 1> ucTimerId:�����ʱ��ID
*
* Reutrn     : 1> 0: ɾ���ɹ�
*              2> ����: �������
*
* Note(s)    : δʹ�õĶ�ʱ�����ܱ�ɾ��
*********************************************************************************************************
*/
int8_t bsp_tim_DeleteSoftTimer(uint8_t ucTimerId)
{
	if(ucTimerId > SOFT_TIMER_COUNT) return -1;
	if(SoftTimer[ucTimerId].ucUsed == 0)  return -2;	
	
	SoftTimer[ucTimerId].v_ucMode = 0;
	SoftTimer[ucTimerId].v_uiPreLoad = 0;
	SoftTimer[ucTimerId].v_uiCount = 0;
		
	return 0;
}
/*
*********************************************************************************************************
*                              bsp_tim_TimerCheck            
*
* Description: ��������ʱ���Ƿ񵽴ﶨʱʱ��
*             
* Arguments  : 1> ucTimerId:��ʱ��ID
*
* Reutrn     : 1> 0: ��ʱʱ�䵽
*              2> 1: δ����ʱʱ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
int8_t bsp_tim_SoftCheck(uint8_t ucTimerId)
{
	if(ucTimerId > SOFT_TIMER_COUNT) return -1;
	
	return (SoftTimer[ucTimerId].v_ucFlag == 1)?0:1;
}

/*
*********************************************************************************************************
*                              bsp_tim_GetRunTime            
*
* Description: ��ȡϵͳ����ʱ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int32_t bsp_tim_SoftGetRunTime(void)
{
	int32_t runTime;
	DISABLE_INT();
	
	runTime = g_iRunTime;
	
	ENABLE_INT();
	
	return runTime;
}


/********************************************  END OF FILE  *******************************************/






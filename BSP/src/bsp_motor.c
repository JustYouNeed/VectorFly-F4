/**
  *******************************************************************************************************
  * File Name: bsp_motor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-5
  * Brief: ���ļ��ṩ���й�С������ĳ�ʼ���Լ���ز�������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-5
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/* 84M��Ƶ�� 8λ�������375K PWM */
#define TIM_CLOCK_HZ 				84000000
#define MOTORS_PWM_BITS           	8
#define MOTORS_PWM_PERIOD         	((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       	0


/*
*********************************************************************************************************
*                           bsp_motor_GPIOConfig               
*
* Description: ��ʼ������Ŀ�������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ���ļ�˽�к���
*********************************************************************************************************
*/
static void bsp_motor_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
*********************************************************************************************************
*                         bsp_motor_PWMConfig                 
*
* Description: ��ʼ�����PWM
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ���ļ�˽�к���
*********************************************************************************************************
*/
static void bsp_motor_PWMConfig(void)
{
		
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ�ܶ�ʱ��1ʱ��
 	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 100;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

  TIM_CtrlPWMOutputs(TIM2,ENABLE);	//MOE �����ʹ��	
	

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	 
	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM1
}


/*
*********************************************************************************************************
*                           bsp_motor_Config               
*
* Description: ��ʼ���������,������������,PWM,�Լ�������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_Config(void)
{
	bsp_motor_GPIOConfig();
	bsp_motor_PWMConfig();
}


/*
*********************************************************************************************************
*                         bsp_motor_SetPWM                 
*
* Description: ���õ����PWM
*             
* Arguments  : 1> LeftPwm: ��ߵ��PWM,����������
*							 2> RightPwm: �ұߵ��PWM,����������
*
* Reutrn     : None
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_motor_SetPWM(int16_t LeftPwm, int16_t RightPwm)
{
	if(LeftPwm == 0) LM_STOP();
	if(RightPwm == 0) RM_STOP();
	/*  ������ߵ��PWM  */
	if(LeftPwm < 0)		/*  ��ת  */
	{
		LM_BACK();
		TIM_SetCompare4(TIM1, -LeftPwm);
	}
	else 
	{
		LM_AHEAD();
		TIM_SetCompare4(TIM1, LeftPwm);
	}
	
	/*  ��ߵ��  */
	if(RightPwm < 0)		/*  ��ת  */
	{
		RM_BACK();
		TIM_SetCompare1(TIM1, 7200 + RightPwm);
	}
	else 
	{
		RM_AHEAD();
		TIM_SetCompare1(TIM1, 7200 - RightPwm);
	}
}

/********************************************  END OF FILE  *******************************************/

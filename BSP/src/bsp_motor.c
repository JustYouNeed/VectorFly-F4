/**
  *******************************************************************************************************
  * File Name: bsp_motor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-5
  * Brief: 本文件提供了有关小车电机的初始化以及相关操作函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-5
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/* 84M主频下 8位精度输出375K PWM */
#define TIM_CLOCK_HZ 				84000000
#define MOTORS_PWM_BITS           	8
#define MOTORS_PWM_PERIOD         	((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       	0


/*
*********************************************************************************************************
*                           bsp_motor_GPIOConfig               
*
* Description: 初始化电机的控制引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 本文件私有函数
*********************************************************************************************************
*/
static void bsp_motor_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
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
* Description: 初始化电机PWM
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 本文件私有函数
*********************************************************************************************************
*/
static void bsp_motor_PWMConfig(void)
{
		
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能定时器1时钟
 	
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 100;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

  TIM_CtrlPWMOutputs(TIM2,ENABLE);	//MOE 主输出使能	
	

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH1预装载使能	 
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH4预装载使能	 
	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM1
}


/*
*********************************************************************************************************
*                           bsp_motor_Config               
*
* Description: 初始化电机部分,包括控制引脚,PWM,以及编码器
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
* Description: 设置电机的PWM
*             
* Arguments  : 1> LeftPwm: 左边电机PWM,正负代表方向
*							 2> RightPwm: 右边电机PWM,正负代表方向
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
	/*  设置左边电机PWM  */
	if(LeftPwm < 0)		/*  反转  */
	{
		LM_BACK();
		TIM_SetCompare4(TIM1, -LeftPwm);
	}
	else 
	{
		LM_AHEAD();
		TIM_SetCompare4(TIM1, LeftPwm);
	}
	
	/*  左边电机  */
	if(RightPwm < 0)		/*  反转  */
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

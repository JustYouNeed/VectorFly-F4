/**
  *******************************************************************************************************
  * File Name: bsp_motor.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-5
  * Brief: 本文件提供了有关电机的数据定义以及函数声明
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-5
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_MOTOR_H
# define __BSP_MOTOR_H


/*  电机控制结构体  */
typedef struct 
{	
	int16_t LeftPwm;				/*  左边电机PWM  */
	int16_t RightPwm;				/*  右边电机PWM  */
	
	int16_t LeftEncoder;		/*  左边电机编码器  */
	int16_t RightEncoder;		/*  右边电机编码器  */
	
	float LeftSpeed;			/*  左边电机转速  */
	float RightSpeed;			/*  右边电机转速  */
}Motor_TypeDef;

/*  电机控制相关引脚  */
# define LM_ENA			PAout(12)
# define LM_ENB			PAout(15)
# define RM_ENA			PBout(4)
# define RM_ENB			PBout(5)

# define LM_BACK()		{LM_ENA = 1; LM_ENB = 0;}
# define LM_AHEAD()		{LM_ENA = 0; LM_ENB = 1;}
# define RM_BACK()		{RM_ENA = 1; RM_ENB = 0;}
# define RM_AHEAD()		{RM_ENA = 0; RM_ENB = 1;}
# define LM_STOP()		{LM_ENA = 1; LM_ENB = 1;}
# define RM_STOP()		{RM_ENA = 1; RM_ENB = 1;}

# define MOTOR_AHEAD() {LM_AHEAD(); RM_AHEAD();}
# define MOTOR_BACK()	 {LM_BACK(); RM_BACK();}
# define MOTOR_TURN_RIGHT()	{LM_AHEAD(); RM_BACK();}
# define MOTOR_TURN_LEFT()	{LM_BACK(); RM_AHEAD();}


/*  电机相关函数  */
void bsp_motor_Config(void);
void bsp_motor_SetPWM(int16_t LeftPwm, int16_t RightPwm);


# endif


/********************************************  END OF FILE  *******************************************/



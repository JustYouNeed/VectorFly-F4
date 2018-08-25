/**
  *******************************************************************************************************
  * File Name: bsp_motor.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-5
  * Brief: ���ļ��ṩ���йص�������ݶ����Լ���������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-5
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_MOTOR_H
# define __BSP_MOTOR_H


/*  ������ƽṹ��  */
typedef struct 
{	
	int16_t LeftPwm;				/*  ��ߵ��PWM  */
	int16_t RightPwm;				/*  �ұߵ��PWM  */
	
	int16_t LeftEncoder;		/*  ��ߵ��������  */
	int16_t RightEncoder;		/*  �ұߵ��������  */
	
	float LeftSpeed;			/*  ��ߵ��ת��  */
	float RightSpeed;			/*  �ұߵ��ת��  */
}Motor_TypeDef;

/*  ��������������  */
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


/*  �����غ���  */
void bsp_motor_Config(void);
void bsp_motor_SetPWM(int16_t LeftPwm, int16_t RightPwm);


# endif


/********************************************  END OF FILE  *******************************************/



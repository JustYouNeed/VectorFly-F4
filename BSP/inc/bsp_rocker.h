/**
  *******************************************************************************************************
  * File Name: bsp_rocker.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-22
  * Brief: ң����ҡ��ģ��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-22
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_ROCKER_H
# define __BSP_ROCKER_H


typedef struct
{
	float LX_Thrust;		/*  ���ҡ��X������  */
	float LY_Thrust;		/*  ���ҡ��y������  */
	float RX_Thrust;		/*  �ұ�ҡ��X������  */
	float RY_Thrust;		/*  �ұ�ҡ��Y������  */
}Rocker_TypeDef;

extern Rocker_TypeDef Rocker;

void bsp_rocker_Config(void);
void bsp_rocker_Handler(void);

# endif
/********************************************  END OF FILE  *******************************************/



/**
  *******************************************************************************************************
  * File Name: bsp_rocker.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-22
  * Brief: 遥控器摇杆模块
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-22
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_ROCKER_H
# define __BSP_ROCKER_H


typedef struct
{
	float LX_Thrust;		/*  左边摇杆X轴推力  */
	float LY_Thrust;		/*  左边摇杆y轴推力  */
	float RX_Thrust;		/*  右边摇杆X轴推力  */
	float RY_Thrust;		/*  右边摇杆Y轴推力  */
}Rocker_TypeDef;

extern Rocker_TypeDef Rocker;

void bsp_rocker_Config(void);
void bsp_rocker_Handler(void);

# endif
/********************************************  END OF FILE  *******************************************/



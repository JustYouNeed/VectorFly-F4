/**
  *******************************************************************************************************
  * File Name: app_sensor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-8-2
  * Brief: VectorFly传感器部分
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-8-2
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
# ifndef __APP_SENSOR_H
# define __APP_SENSOR_H


#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
}Axis3i16_t;

typedef union
{
	struct
	{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	int32_t axis[3];
}Axis3i32_t;

typedef union
{
	struct 
	{
		int64_t x;
		int64_t y;
		int64_t z;
	};
	int64_t axis[3];
}Axis3i64_t;

typedef union
{
	struct
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
}Axis3f_t;

typedef struct 
{
	Axis3f_t acc;
	Axis3f_t gyro;
	Axis3f_t mag;
	
}sensorData_t;


void sensors_Task(void *p_arg);
void sensors_Init(void);
bool sensors_Test(void);

void sensors_AcquireData(sensorData_t *sensors);

bool sensors_GetGyroData(Axis3f_t *gyro);
bool sensors_GetAccData(Axis3f_t *acc);
bool sensors_GetMagData(Axis3f_t *mag);
bool sensors_GetBaroData(void);


# endif

/********************************************  END OF FILE  *******************************************/



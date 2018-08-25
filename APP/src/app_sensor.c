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
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"


#define SENSORS_GYRO_FS_CFG       MPU_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

typedef struct
{
	Axis3f_t bias;
	bool isBiasValueFound;
	bool isBufferFilled;
	Axis3i16_t *bufHead;
	Axis3i16_t buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
/*  传感器数据准备好标志量  */
static xSemaphoreHandle	sensorsDataReady;
static xQueueHandle	accDataQueue;		/*  加速度数据队列  */
static xQueueHandle	gyroDataQueue;	/*  角速度数据队列  */
static xQueueHandle	magDataQueue;		/*  磁场数据队列  */
static xQueueHandle	baroDataQueue;	/*  气压计数据队列  */

/*  是否初始化标志量  */
static bool isInit = false;
static Axis3i16_t	acc, gyro, mag;
static sensorData_t sensors;

/*  用于传感器校准  */
static BiasObj	gyroBiasRunning;
static Axis3f_t gyroBias;
static bool	isGyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1.0f;

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData_t accLpf[3];
static lpf2pData_t gyroLpf[3];


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
bool sensors_GetGyroData(Axis3f_t *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
bool sensors_GetAccData(Axis3f_t *acc)
{
	return (pdTRUE == xQueueReceive(accDataQueue, acc, 0));
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
bool sensors_GetMagData(Axis3f_t *mag)
{
	return (pdTRUE == xQueueReceive(magDataQueue, mag, 0));
}


void sensors_AcquireData(sensorData_t *sensors)
{
	sensors_GetGyroData(&sensors->gyro);
	sensors_GetAccData(&sensors->acc);
	sensors_GetMagData(&sensors->mag);
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
static void sensors_Axis3fLpf(lpf2pData_t *data, Axis3f_t *in)
{
	for(uint8_t i = 0; i < 3; i ++)
	{
		in->axis[i] = filter_LPF2P_Filter(&data[i], in->axis[i]);
	}
}


/*
*********************************************************************************************************
*                       sensors_DeviceInit                   
*
* Description: 初始化传感器器件
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void sensors_DeviceInit(void)
{
	/*  创建传感器消息队列  */
	accDataQueue = xQueueCreate(1, sizeof(Axis3f_t));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f_t));
	magDataQueue = xQueueCreate(1, sizeof(Axis3f_t));
	
	
	bsp_mpu_Init();

	vTaskDelay(10);
	bsp_mpu_Reset();
	vTaskDelay(20);
	
	bsp_mpu_SetSleepEnabled(false);
	vTaskDelay(10);
	bsp_mpu_SetClockSource(MPU_CLOCK_PLL_XGYRO);
	vTaskDelay(10);
	bsp_mpu_SetTempSensorEnabled(true);	// 使能温度传感器	
	bsp_mpu_SetIntEnabled(false);		// 关闭中断	
	bsp_mpu_SetI2CBypassEnabled(true);	// 旁路模式，磁力计和气压连接到主IIC	
	bsp_mpu_SetFullScaleGyroRange(MPU_GYRO_FS_2000);	// 设置陀螺量程	
	bsp_mpu_SetFullScaleAccelRange(MPU_ACCEL_FS_16);// 设置加速计量程	
	bsp_mpu_SetAccelDLPF(MPU_ACCEL_DLPF_BW_41);		// 设置加速计数字低通滤波

	bsp_mpu_SetRate(0);// 设置采样速率: 1000 / (1 + 0) = 1000Hz
	bsp_mpu_SetDLPFMode(MPU_DLPF_BW_98);// 设置陀螺数字低通滤波
	
	bsp_mag_Init();
	bsp_mag_SetMode(MAG_MODE_16BIT | MAG_MODE_CONT2);
	
	
	for(uint8_t i = 0; i < 3; i++)
	{
		filter_LPF2P_Init(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		filter_LPF2P_Init(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
	}
}


/*
*********************************************************************************************************
*                       sensors_BiasObjInit                   
*
* Description: 传感器偏置初始化
*             
* Arguments  : 1.bias: 传感器偏置结构体
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void sensors_BiasObjInit(BiasObj *bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

/*
*********************************************************************************************************
*                         sensors_CalcVarianceAndMean                 
*
* Description: 计算方差还有平均值
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void sensors_CalcVarianceAndMean(BiasObj *bias, Axis3f_t *varOut, Axis3f_t *meanOut)
{
	uint32_t i = 0;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};
	
	for(i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i ++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}
	
	varOut->x = (sumsq[0] - ((int64_t)sum[0]*sum[0])) / SENSORS_NBR_OF_BIAS_SAMPLES;
	varOut->y = (sumsq[1] - ((int64_t)sum[1]*sum[1])) / SENSORS_NBR_OF_BIAS_SAMPLES;
	varOut->z = (sumsq[2] - ((int64_t)sum[2]*sum[2])) / SENSORS_NBR_OF_BIAS_SAMPLES;
	
	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

static bool sensors_FindBiasValue(BiasObj *bias)
{
	bool isBiasFound = false;
	
	/*  如果采样数据已经足够,就可以进行偏差的计算  */
	if(bias->isBufferFilled)
	{
		Axis3f_t variance;
		Axis3f_t mean;
		
		sensors_CalcVarianceAndMean(bias, &variance, &mean);
		
		/*  偏差有一个合理的范围  */
		if(variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			isBiasFound = true;
			bias->isBiasValueFound = true;
		}
		else
			bias->isBufferFilled = false;
	}
	
	return isBiasFound;
}
	


/*
*********************************************************************************************************
*                             sensors_InterruptInit             
*
* Description: 初始化MPU传感器的外部中断
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void sensors_InterruptInit(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = MPU_INT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MPU_INT_PORT, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	portDISABLE_INTERRUPTS();
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line5);
	portENABLE_INTERRUPTS();
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}
/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void sensors_Init(void)
{
	if(isInit) return ;
	
	sensorsDataReady = xSemaphoreCreateBinary();
	sensors_BiasObjInit(&gyroBiasRunning);
	sensors_DeviceInit();
	sensors_InterruptInit();
	isInit = true;
}

/*
*********************************************************************************************************
*                        sensors_SetupSalveRead                  
*
* Description: 设置传感器从模式读取,将磁力计,气压计挂载到MPU上面
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void sensors_SetupSalveRead(void)
{
	bsp_mpu_SetSlave4MasterDelay(9);
	bsp_mpu_SetI2CBypassEnabled(false);
	bsp_mpu_SetInterruptMode(0);
	bsp_mpu_SetInterruptDrive(0); 		// 推挽输出
	bsp_mpu_SetInterruptLatch(0); 		// 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
	bsp_mpu_SetInterruptLatchClear(1); 	// 中断清除模式(0=status-read-only, 1=any-register-read)
	bsp_mpu_SetSlaveReadWriteTransitionEnabled(false); // 关闭从机读写传输
	bsp_mpu_SetMasterClockSpeed(13); 	// 设置i2c速度400kHz
	
	
	// 设置MPU6500主机要读取的寄存器
	bsp_mpu_SetSlaveAddress(0, 0x80 | MAG_ADDRESS_00); 	// 设置磁力计为0号从机
	bsp_mpu_SetSlaveRegister(0, MAG_RA_ST1); 				// 从机0需要读取的寄存器
	bsp_mpu_SetSlaveDataLength(0, 8); 	// 读取8个字节(ST1, x, y, z heading, ST2 (overflow check))
	bsp_mpu_SetSlaveDelayEnabled(0, true);
	bsp_mpu_SetSlaveEnabled(0, true);
	
	bsp_mpu_SetI2CMasterModeEnabled(true);	//使能mpu6500主机模式
	bsp_mpu_SetIntDataReadyEnabled(true);	//数据就绪中断使能
}


/*
*********************************************************************************************************
*                        sensors_AddBiasValue                  
*
* Description: 往偏置结构体里面添加新值,如果缓冲区满了则替换旧值
*             
* Arguments  : 1.bias: 偏置结构体
*							 2.var: 数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void sensors_AddBiasValue(BiasObj *bias, Axis3i16_t var)
{
	bias->bufHead->x = var.x;
	bias->bufHead->y = var.y;
	bias->bufHead->z = var.z;
	bias->bufHead++;
	
	if(bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}
/*
*********************************************************************************************************
*                       sensors_ProcessAccScale                   
*
* Description: 根据样本计算重力加速度缩放因子
*             
* Arguments  : 1.acc: 加速度数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static bool sensors_ProcessAccScale(Axis3i16_t acc)
{
	static bool isAccBiasFound = false;
	static uint32_t accScaleSumCount = 0;
	
	if(!isAccBiasFound)
	{
		accScaleSum += sqrtf(powf(acc.x * SENSORS_G_PER_LSB_CFG, 2) + powf(acc.y * SENSORS_G_PER_LSB_CFG, 2) + powf(acc.z * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;
		
		if(accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			isAccBiasFound = true;
		}
	}
	
	return isAccBiasFound;;
}

/*
*********************************************************************************************************
*                          sensors_ProcessGyroBias                
*
* Description: 计算陀螺仪偏置
*             
* Arguments  : 1.gyro: 陀螺仪数据输入
*							 2.gyroBiasOut: 陀螺仪偏置数据输出
*
* Reutrn     : 是否找到陀螺仪偏置
*
* Note(s)    : None.
*********************************************************************************************************
*/
static bool sensors_ProcessGyroBias(Axis3i16_t gyro, Axis3f_t *gyroBiasOut)
{
	sensors_AddBiasValue(&gyroBiasRunning, gyro);
	
	if(!gyroBiasRunning.isBiasValueFound)
	{
		sensors_FindBiasValue(&gyroBiasRunning);
		
		if(gyroBiasRunning.isBiasValueFound)
		{
			/*  传感器校准成功灯闪  */
		}
	}
	
	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;
	
	return gyroBiasRunning.isBiasValueFound;
}

/*
*********************************************************************************************************
*                          sensors_ProcessMagnetomterData                
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void sensors_ProcessMagnetomterData(const Axis3i16_t mag)
{
	sensors.mag.x = (float)mag.x / MAG_GAUSS_PER_LSB;
	sensors.mag.y = (float)mag.y / MAG_GAUSS_PER_LSB;
	sensors.mag.z = (float)mag.z / MAG_GAUSS_PER_LSB;
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void sensors_ProcessAccAndGyroData(const Axis3i16_t acc, const Axis3i16_t gyro)
{	
	isGyroBiasFound = sensors_ProcessGyroBias(gyro, &gyroBias);
	if(isGyroBiasFound)
	{
		sensors_ProcessAccScale(acc);
	}
	
	sensors.gyro.x = -(gyro.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.y =  (gyro.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gyro.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	sensors_Axis3fLpf((lpf2pData_t *)(&gyroLpf), &sensors.gyro);
	
	sensors.acc.x = -(acc.x) * SENSORS_G_PER_LSB_CFG * accScale;
	sensors.acc.y =  (acc.y) * SENSORS_G_PER_LSB_CFG * accScale;
	sensors.acc.z =  (acc.z) * SENSORS_G_PER_LSB_CFG * accScale;
	sensors_Axis3fLpf((lpf2pData_t *)(&accLpf), &sensors.acc);
}
/*
*********************************************************************************************************
*                           sensorTask               
*
* Description: 传感器任务,负责读取传感器数据并发送到传感器消息队列
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void sensors_Task(void *p_arg)
{
	sensors_Init();
	vTaskDelay(100);
	sensors_SetupSalveRead();
	
	while(1)
	{
		/*  传感器数据准备好后就可以读取  */
		if(pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
			bsp_mpu_GetMotion9(&acc.x, &acc.y, &acc.z, &gyro.x, &gyro.y, &gyro.z, &mag.x, &mag.y, &mag.z);
			
			sensors_ProcessAccAndGyroData(acc, gyro);
			sensors_ProcessMagnetomterData(mag);
			
			vTaskSuspendAll();
			xQueueOverwrite(accDataQueue, &sensors.acc);
			xQueueOverwrite(gyroDataQueue, &sensors.gyro);
			xQueueOverwrite(magDataQueue, &sensors.mag);
			xTaskResumeAll();
		}
	}
}

/*
*********************************************************************************************************
*                           sensors_Isr               
*
* Description: MPU中断处理函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void __attribute__((used)) sensors_Isr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);
	
	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}

/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void __attribute__((used)) EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		sensors_Isr();
	}
}



/********************************************  END OF FILE  *******************************************/



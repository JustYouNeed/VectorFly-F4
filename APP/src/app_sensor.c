/**
  *******************************************************************************************************
  * File Name: app_sensor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-8-2
  * Brief: VectorFly����������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-8-2
	*			Mod: �����ļ�
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

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* ���㷽��Ĳ����������� */
#define GYRO_VARIANCE_BASE				4000	/* ��������ƫ������ֵ */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* ���ټƲ������� */

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
/*  ����������׼���ñ�־��  */
static xSemaphoreHandle	sensorsDataReady;
static xQueueHandle	accDataQueue;		/*  ���ٶ����ݶ���  */
static xQueueHandle	gyroDataQueue;	/*  ���ٶ����ݶ���  */
static xQueueHandle	magDataQueue;		/*  �ų����ݶ���  */
static xQueueHandle	baroDataQueue;	/*  ��ѹ�����ݶ���  */

/*  �Ƿ��ʼ����־��  */
static bool isInit = false;
static Axis3i16_t	acc, gyro, mag;
static sensorData_t sensors;

/*  ���ڴ�����У׼  */
static BiasObj	gyroBiasRunning;
static Axis3f_t gyroBias;
static bool	isGyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1.0f;

/*��ͨ�˲�����*/
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
* Description: ��ʼ������������
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
	/*  ������������Ϣ����  */
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
	bsp_mpu_SetTempSensorEnabled(true);	// ʹ���¶ȴ�����	
	bsp_mpu_SetIntEnabled(false);		// �ر��ж�	
	bsp_mpu_SetI2CBypassEnabled(true);	// ��·ģʽ�������ƺ���ѹ���ӵ���IIC	
	bsp_mpu_SetFullScaleGyroRange(MPU_GYRO_FS_2000);	// ������������	
	bsp_mpu_SetFullScaleAccelRange(MPU_ACCEL_FS_16);// ���ü��ټ�����	
	bsp_mpu_SetAccelDLPF(MPU_ACCEL_DLPF_BW_41);		// ���ü��ټ����ֵ�ͨ�˲�

	bsp_mpu_SetRate(0);// ���ò�������: 1000 / (1 + 0) = 1000Hz
	bsp_mpu_SetDLPFMode(MPU_DLPF_BW_98);// �����������ֵ�ͨ�˲�
	
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
* Description: ������ƫ�ó�ʼ��
*             
* Arguments  : 1.bias: ������ƫ�ýṹ��
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
* Description: ���㷽���ƽ��ֵ
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
	
	/*  ������������Ѿ��㹻,�Ϳ��Խ���ƫ��ļ���  */
	if(bias->isBufferFilled)
	{
		Axis3f_t variance;
		Axis3f_t mean;
		
		sensors_CalcVarianceAndMean(bias, &variance, &mean);
		
		/*  ƫ����һ������ķ�Χ  */
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
* Description: ��ʼ��MPU���������ⲿ�ж�
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
* Description: ���ô�������ģʽ��ȡ,��������,��ѹ�ƹ��ص�MPU����
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
	bsp_mpu_SetInterruptDrive(0); 		// �������
	bsp_mpu_SetInterruptLatch(0); 		// �ж�����ģʽ(0=50us-pulse, 1=latch-until-int-cleared)
	bsp_mpu_SetInterruptLatchClear(1); 	// �ж����ģʽ(0=status-read-only, 1=any-register-read)
	bsp_mpu_SetSlaveReadWriteTransitionEnabled(false); // �رմӻ���д����
	bsp_mpu_SetMasterClockSpeed(13); 	// ����i2c�ٶ�400kHz
	
	
	// ����MPU6500����Ҫ��ȡ�ļĴ���
	bsp_mpu_SetSlaveAddress(0, 0x80 | MAG_ADDRESS_00); 	// ���ô�����Ϊ0�Ŵӻ�
	bsp_mpu_SetSlaveRegister(0, MAG_RA_ST1); 				// �ӻ�0��Ҫ��ȡ�ļĴ���
	bsp_mpu_SetSlaveDataLength(0, 8); 	// ��ȡ8���ֽ�(ST1, x, y, z heading, ST2 (overflow check))
	bsp_mpu_SetSlaveDelayEnabled(0, true);
	bsp_mpu_SetSlaveEnabled(0, true);
	
	bsp_mpu_SetI2CMasterModeEnabled(true);	//ʹ��mpu6500����ģʽ
	bsp_mpu_SetIntDataReadyEnabled(true);	//���ݾ����ж�ʹ��
}


/*
*********************************************************************************************************
*                        sensors_AddBiasValue                  
*
* Description: ��ƫ�ýṹ�����������ֵ,����������������滻��ֵ
*             
* Arguments  : 1.bias: ƫ�ýṹ��
*							 2.var: ����
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
* Description: �������������������ٶ���������
*             
* Arguments  : 1.acc: ���ٶ�����
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
* Description: ����������ƫ��
*             
* Arguments  : 1.gyro: ��������������
*							 2.gyroBiasOut: ������ƫ���������
*
* Reutrn     : �Ƿ��ҵ�������ƫ��
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
			/*  ������У׼�ɹ�����  */
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
* Description: ����������,�����ȡ���������ݲ����͵���������Ϣ����
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
		/*  ����������׼���ú�Ϳ��Զ�ȡ  */
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
* Description: MPU�жϴ�����
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



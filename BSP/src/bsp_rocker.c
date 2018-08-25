/**
  *******************************************************************************************************
  * File Name: bsp_rocker.c
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
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"


/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static uint16_t AD_Digital_Value[4] = {0,0,0,0};

Rocker_TypeDef Rocker;

/*
*********************************************************************************************************
*                           bsp_rocker_GPIOConfig               
*
* Description: 初始化摇杆引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_rocker_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}

/*
*********************************************************************************************************
*                                bsp_rocker_ADCConfig          
*
* Description: 初始化摇杆ADC部分
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_rocker_ADCConfig(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_DeInit(ADC1);		

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //设置ADC工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ;  //扫描通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//工作在连续模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	 //软件触发来模数转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	  //数据向右边对齐
	ADC_InitStructure.ADC_NbrOfChannel = 4;	  //规则序列的长度，ADC通道的数目1（范围为1~16）
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_71Cycles5 );	//配置采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_71Cycles5 );	//配置采样时间为239.5周期	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_71Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_71Cycles5 );
	
	
	ADC_DMACmd(ADC1, ENABLE); //使能ADC1的DMA传输 ，原本ADC上没有的，注意添加

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);		//重置指定的ADC的校准寄存器
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));	   //获取ADC重置校准寄存器的状态
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);		 //开始指定ADC的校准状态
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));	//等待校准AD结束
}

/*
*********************************************************************************************************
*                                bsp_rocker_DMAConfig          
*
* Description: 初始化摇杆ADC的DMA部分
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_rocker_DMAConfig(void)
{
	DMA_InitTypeDef DMA_InitStructure;

 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA时钟
	
	DMA_DeInit(DMA1_Channel1);   //将DMA的通道1寄存器重设为缺省值

	//DMA1_MEM_LEN=cndtr;//保存通道数据长度
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)AD_Digital_Value;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设发送到内存  DMA_CCRX位4
	DMA_InitStructure.DMA_BufferSize = 4;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //外设数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //内存数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
}

/*
*********************************************************************************************************
*                           bsp_rocker_Config               
*
* Description: 初始化摇杆部分
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_rocker_Config(void)
{
	bsp_rocker_GPIOConfig();
	bsp_rocker_ADCConfig();
	bsp_rocker_DMAConfig();
	
	DMA_Cmd(DMA1_Channel1,ENABLE);
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
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
void bsp_rocker_Handler(void)
{
	/*  左边X轴  */
	Rocker.LX_Thrust = (float)((((AD_Digital_Value[0] >> 4) * 3.3) / 255) - 1.436f);
	
	if(Rocker.LX_Thrust < 0) Rocker.LX_Thrust = ((Rocker.LX_Thrust) / 1.436f) * 100;
	else Rocker.LX_Thrust = ((Rocker.LX_Thrust) / 1.85f) * 100;
	if(Rocker.LX_Thrust < 2 && Rocker.LX_Thrust > -2) Rocker.LX_Thrust = 0;
	
	/*  左边Y轴  */
	Rocker.LY_Thrust = (float)((((AD_Digital_Value[1] >> 4) * 3.3) / 255) - 1.736f);
	
	if(Rocker.LY_Thrust < 0) Rocker.LY_Thrust = ((Rocker.LY_Thrust) / 1.436f) * 100;
	else Rocker.LY_Thrust = ((Rocker.LY_Thrust) / 1.85f) * 100;
	if(Rocker.LY_Thrust < 2 && Rocker.LY_Thrust > -2) Rocker.LY_Thrust = 0;
	
	/*  右边X轴  */
	Rocker.RX_Thrust = (float)((((AD_Digital_Value[2] >> 4) * 3.3) / 255) - 1.636f);
	
	if(Rocker.RX_Thrust < 0) Rocker.RX_Thrust = ((Rocker.RX_Thrust) / 1.436f) * 100;
	else Rocker.RX_Thrust = ((Rocker.RX_Thrust) / 1.85f) * 100;
	if(Rocker.RX_Thrust < 2 && Rocker.RX_Thrust > -2) Rocker.RX_Thrust = 0;
	
	/*  右边Y轴  */
	Rocker.RY_Thrust = (float)((((AD_Digital_Value[3] >> 4) * 3.3) / 255) - 1.836f);
	
	if(Rocker.RY_Thrust < 0) Rocker.RY_Thrust = ((Rocker.RY_Thrust) / 1.436f) * 100;
	else Rocker.RY_Thrust = ((Rocker.RY_Thrust) / 1.85f) * 100;
	if(Rocker.RY_Thrust < 2 && Rocker.RY_Thrust > -2) Rocker.RY_Thrust = 0;
	
}

/********************************************  END OF FILE  *******************************************/


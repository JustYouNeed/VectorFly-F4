/**
  *******************************************************************************************************
  * File Name: bsp_rocker.c
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
* Description: ��ʼ��ҡ������
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
* Description: ��ʼ��ҡ��ADC����
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

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //����ADC�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ;  //ɨ��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//����������ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	 //���������ģ��ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	  //�������ұ߶���
	ADC_InitStructure.ADC_NbrOfChannel = 4;	  //�������еĳ��ȣ�ADCͨ������Ŀ1����ΧΪ1~16��
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_71Cycles5 );	//���ò���ʱ��Ϊ239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_71Cycles5 );	//���ò���ʱ��Ϊ239.5����	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_71Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_71Cycles5 );
	
	
	ADC_DMACmd(ADC1, ENABLE); //ʹ��ADC1��DMA���� ��ԭ��ADC��û�еģ�ע�����

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);		//����ָ����ADC��У׼�Ĵ���
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));	   //��ȡADC����У׼�Ĵ�����״̬
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);		 //��ʼָ��ADC��У׼״̬
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));	//�ȴ�У׼AD����
}

/*
*********************************************************************************************************
*                                bsp_rocker_DMAConfig          
*
* Description: ��ʼ��ҡ��ADC��DMA����
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

 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMAʱ��
	
	DMA_DeInit(DMA1_Channel1);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	//DMA1_MEM_LEN=cndtr;//����ͨ�����ݳ���
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)AD_Digital_Value;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴����跢�͵��ڴ�  DMA_CCRXλ4
	DMA_InitStructure.DMA_BufferSize = 4;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //�������ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //�ڴ����ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
}

/*
*********************************************************************************************************
*                           bsp_rocker_Config               
*
* Description: ��ʼ��ҡ�˲���
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
	/*  ���X��  */
	Rocker.LX_Thrust = (float)((((AD_Digital_Value[0] >> 4) * 3.3) / 255) - 1.436f);
	
	if(Rocker.LX_Thrust < 0) Rocker.LX_Thrust = ((Rocker.LX_Thrust) / 1.436f) * 100;
	else Rocker.LX_Thrust = ((Rocker.LX_Thrust) / 1.85f) * 100;
	if(Rocker.LX_Thrust < 2 && Rocker.LX_Thrust > -2) Rocker.LX_Thrust = 0;
	
	/*  ���Y��  */
	Rocker.LY_Thrust = (float)((((AD_Digital_Value[1] >> 4) * 3.3) / 255) - 1.736f);
	
	if(Rocker.LY_Thrust < 0) Rocker.LY_Thrust = ((Rocker.LY_Thrust) / 1.436f) * 100;
	else Rocker.LY_Thrust = ((Rocker.LY_Thrust) / 1.85f) * 100;
	if(Rocker.LY_Thrust < 2 && Rocker.LY_Thrust > -2) Rocker.LY_Thrust = 0;
	
	/*  �ұ�X��  */
	Rocker.RX_Thrust = (float)((((AD_Digital_Value[2] >> 4) * 3.3) / 255) - 1.636f);
	
	if(Rocker.RX_Thrust < 0) Rocker.RX_Thrust = ((Rocker.RX_Thrust) / 1.436f) * 100;
	else Rocker.RX_Thrust = ((Rocker.RX_Thrust) / 1.85f) * 100;
	if(Rocker.RX_Thrust < 2 && Rocker.RX_Thrust > -2) Rocker.RX_Thrust = 0;
	
	/*  �ұ�Y��  */
	Rocker.RY_Thrust = (float)((((AD_Digital_Value[3] >> 4) * 3.3) / 255) - 1.836f);
	
	if(Rocker.RY_Thrust < 0) Rocker.RY_Thrust = ((Rocker.RY_Thrust) / 1.436f) * 100;
	else Rocker.RY_Thrust = ((Rocker.RY_Thrust) / 1.85f) * 100;
	if(Rocker.RY_Thrust < 2 && Rocker.RY_Thrust > -2) Rocker.RY_Thrust = 0;
	
}

/********************************************  END OF FILE  *******************************************/


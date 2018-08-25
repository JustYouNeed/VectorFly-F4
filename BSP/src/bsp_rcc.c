/**
  *******************************************************************************************************
  * File Name: bsp_rcc.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-5-26
  * Brief: ����ʱ�ӹ���ģ��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-5-26
	*     Mod: �����ļ�
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# define GET_APB1_RCC(RCCx)	RCC_APB1Periph_##RCCx
# define GET_APB2_RCC(RCCx) RCC_APB2Periph_##RCCx
# define GET_SPI_AF(SPIx)		GPIO_AF_##SPIx
# define GET_TIM_AF(TIMx)		GPIO_AF_##TIMx

# define GET_ADC_RCC(ADCx)	GET_APB1_RCC(ADCx)

/*
*********************************************************************************************************
*                                   bsp_GetTIMRCC       
*
* Description: ��ȡ��ʱ������ʱ�ӱ�������
*             
* Arguments  : TIMx:Ҫ��ȡ��ʱ��ʱ�ӵĶ�ʱ����ţ�ΪTIM_TypeDef�ṹ��ָ��
*
* Reutrn     ������ֵΪ��ʱ������ʱ�ӱ���
*
* Note(s)    : 
*********************************************************************************************************
*/
uint32_t bsp_rcc_GetTIMRCC(TIM_TypeDef* TIMx)
{
	uint32_t rcc = 0;
	
//	if((TIMx == TIM2) || (TIMx == TIM3) || (TIMx == TIM4) || (TIMx == TIM5) || (TIMx == TIM6) || (TIMx == TIM7) ||
//		 (TIMx == TIM12) || (TIMx == TIM13) || (TIMx == TIM14))
//		rcc = GET_APB1_RCC(TIMx);
//	else
//		rcc = GET_APB2_RCC(TIMx);
	
	if(TIMx == TIM1)	rcc = RCC_APB2Periph_TIM1;
	else if(TIMx == TIM2) rcc = RCC_APB1Periph_TIM2;
	else if(TIMx == TIM3) rcc = RCC_APB1Periph_TIM3;
	else if(TIMx == TIM4) rcc = RCC_APB1Periph_TIM4;
	else if(TIMx == TIM5) rcc = RCC_APB1Periph_TIM5;
	else if(TIMx == TIM6) rcc = RCC_APB1Periph_TIM6;
	else if(TIMx == TIM7) rcc = RCC_APB1Periph_TIM7;
	else if(TIMx == TIM8) rcc = RCC_APB2Periph_TIM8;
	else if(TIMx == TIM9) rcc = RCC_APB2Periph_TIM9;
	else if(TIMx == TIM10) rcc = RCC_APB2Periph_TIM10;
	else if(TIMx == TIM11) rcc = RCC_APB2Periph_TIM11;
	else if(TIMx == TIM12) rcc = RCC_APB1Periph_TIM12;
	else if(TIMx == TIM13) rcc = RCC_APB1Periph_TIM13;
	else if(TIMx == TIM14) rcc = RCC_APB1Periph_TIM14;
	
	return rcc;
}

/*
*********************************************************************************************************
*                                      bsp_GetGPIORCC    
*
* Description: ��ȡGPIO����ʱ�ӱ���
*             
* Arguments : GPIOx:Ҫ��ȡʱ�ӱ�����GPIO�˿ڣ�ΪGPIO_TypeDef�ṹ��ָ��
*
* Reutrn��
*
* Note(s)   : ����ֵΪ��Ӧ��GPIO�˿�ʱ�ӱ���
*********************************************************************************************************
*/
uint32_t bsp_rcc_GetGPIORCC(GPIO_TypeDef* GPIOx)
{
	uint32_t rcc = 0;
	
# ifdef VECTOR_F1
	if(GPIOx == GPIOA) rcc = RCC_APB2Periph_GPIOA;
	else if(GPIOx == GPIOB) rcc = RCC_APB2Periph_GPIOB;
	else if(GPIOx == GPIOC) rcc = RCC_APB2Periph_GPIOC;
	else if(GPIOx == GPIOD) rcc = RCC_APB2Periph_GPIOD;
	else if(GPIOx == GPIOE) rcc = RCC_APB2Periph_GPIOE;
	else if(GPIOx == GPIOF) rcc = RCC_APB2Periph_GPIOF;
	else if(GPIOx == GPIOG) rcc = RCC_APB2Periph_GPIOG;
# elif defined VECTOR_F4
	if(GPIOx == GPIOA) rcc = RCC_AHB1Periph_GPIOA;
	else if(GPIOx == GPIOB) rcc = RCC_AHB1Periph_GPIOB;
	else if(GPIOx == GPIOC) rcc = RCC_AHB1Periph_GPIOC;
	else if(GPIOx == GPIOD) rcc = RCC_AHB1Periph_GPIOD;
	else if(GPIOx == GPIOE) rcc = RCC_AHB1Periph_GPIOE;
	else if(GPIOx == GPIOF) rcc = RCC_AHB1Periph_GPIOF;
	else if(GPIOx == GPIOH) rcc = RCC_AHB1Periph_GPIOH;
	else if(GPIOx == GPIOI) rcc = RCC_AHB1Periph_GPIOI;
	else if(GPIOx == GPIOG) rcc = RCC_AHB1Periph_GPIOG;
# endif
	return rcc;
}


uint32_t bsp_rcc_GetADCRCC(ADC_TypeDef* ADCx)
{
	uint32_t rcc = 0;
	if(ADC1 == ADCx) rcc = RCC_APB2Periph_ADC1;
	else if(ADC2 == ADCx) rcc = RCC_APB2Periph_ADC2;
	else if(ADC3 == ADCx) rcc = RCC_APB2Periph_ADC3;
	return rcc;
}

/*
*********************************************************************************************************
*                                       bsp_GetTIMPinAF   
*
* Description: ����GPIO���óɶ�ʱ��ģʽʱ������ͨ���ú�����ȡ���ö�ʱ��
*             
* Arguments  : TIMx:��ʱ�����
*
* Reutrn     : ���Ÿ���Ϊ��ʱ���ı��
*
* Note(s)    : 
*********************************************************************************************************
*/
# ifdef VECTOR_F4
uint8_t bsp_GetTIMPinAF(TIM_TypeDef *TIMx)
{
	uint8_t pinAF = 0;
	if(TIMx == TIM1)	pinAF = GPIO_AF_TIM1;
	else if(TIMx == TIM2) pinAF = GPIO_AF_TIM2;
	else if(TIMx == TIM3) pinAF = GPIO_AF_TIM3;
	else if(TIMx == TIM4) pinAF = GPIO_AF_TIM4;
	else if(TIMx == TIM5) pinAF = GPIO_AF_TIM5;
	else if(TIMx == TIM8) pinAF = GPIO_AF_TIM8;
	else if(TIMx == TIM9) pinAF = GPIO_AF_TIM9;
	else if(TIMx == TIM10) pinAF = GPIO_AF_TIM10;
	else if(TIMx == TIM11) pinAF = GPIO_AF_TIM11;
	else if(TIMx == TIM12) pinAF = GPIO_AF_TIM12;
	else if(TIMx == TIM13) pinAF = GPIO_AF_TIM13;
	else if(TIMx == TIM14) pinAF = GPIO_AF_TIM14;
	
	return pinAF;
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
uint8_t bsp_GetSPIPinAF(SPI_TypeDef* SPIx)
{
	uint8_t pinAF = 0;
	if(SPIx == SPI1)	pinAF = GPIO_AF_SPI1;
	else if(SPIx == SPI2) pinAF = GPIO_AF_SPI2;
	else if(SPIx == SPI3) pinAF = GPIO_AF_SPI3;
	else if(SPIx == SPI4) pinAF = GPIO_AF_SPI4;
	else if(SPIx == SPI5) pinAF = GPIO_AF_SPI5;
	else if(SPIx == SPI6) pinAF = GPIO_AF_SPI6;
	
	return pinAF;
}


# endif
/*
*********************************************************************************************************
*                                         bsp_GetPinSource 
*
* Description: ��ȡGPIO������Դ
*             
* Arguments  : GPIO_Pin:GPIO���ű�ţ�ֻ�����뵥������
*
* Reutrn     : ��Ӧ������Դ
*
* Note(s)    : ��
*********************************************************************************************************
*/
uint16_t bsp_GetPinSource(uint16_t GPIO_Pin)
{
	uint16_t PinSource = 0;
	
	if(GPIO_Pin > GPIO_Pin_15) return 0xff;
	
	for(PinSource = 0; PinSource < 15; PinSource ++)
	{
		if((GPIO_Pin >> PinSource) & 0x01) break;
	}
	
	return PinSource;
}


/*
*********************************************************************************************************
*                                          bsp_TIMClockCmd
*
* Description: ��ʱ������ʱ��ʹ�ܺ���
*             
* Arguments  : TIMx:��ʱ����ţ�ΪTIM_TypeDef�ṹ��ָ��
*
* Reutrn     : ��
*
* Note(s)    : ��
*********************************************************************************************************
*/
void bsp_rcc_TIMClockCmd(TIM_TypeDef *TIMx)
{
	if(TIMx == TIM2 || TIMx == TIM3) RCC_APB1PeriphClockCmd(bsp_rcc_GetTIMRCC(TIMx), ENABLE);
# if defined (STM32F10X_HD)|| defined (STM32F10X_MD) || defined (VECTOR_F4)
	else if(TIMx == TIM4 || TIMx == TIM5 || TIMx == TIM6) RCC_APB1PeriphClockCmd(bsp_rcc_GetTIMRCC(TIMx), ENABLE);
# endif
	
# ifdef VECTOR_F4
	else if(TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14 ) RCC_APB1PeriphClockCmd(bsp_rcc_GetTIMRCC(TIMx), ENABLE);
	else if(TIMx == TIM8 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
		 RCC_APB2PeriphClockCmd(bsp_rcc_GetTIMRCC(TIMx), ENABLE);
# endif
		
}


/*
*********************************************************************************************************
*                                          bsp_GPIOClcokCmd
*
* Description: GPIO�˿�ʱ��ʹ�ܺ���
*             
* Arguments  : GPIOx:Ҫʹ�ܵ�GPIO�˿�
*
* Reutrn     : ��
*
* Note(s)    : ��
*********************************************************************************************************
*/
void bsp_rcc_GPIOClcokCmd(GPIO_TypeDef* GPIOx)
{
# ifdef VECTOR_F1
	RCC_APB2PeriphClockCmd(bsp_rcc_GetGPIORCC(GPIOx), ENABLE);
# elif defined VECTOR_F4
	RCC_AHB1PeriphClockCmd(bsp_rcc_GetGPIORCC(GPIOx), ENABLE);
# endif
}



/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments : 
*
* Reutrn:
*
* Note(s)   : 
*********************************************************************************************************
*/
void bsp_rcc_ADCClockCmd(ADC_TypeDef* ADCx)
{
	RCC_APB2PeriphClockCmd(bsp_rcc_GetADCRCC(ADCx), ENABLE);
}




/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments : 
*
* Reutrn:
*
* Note(s)   : 
*********************************************************************************************************
*/
uint32_t bsp_GetSPIRCC(SPI_TypeDef *SPIx)
{
	uint32_t rcc = 0;
	if(SPIx == SPI1) rcc = RCC_APB2Periph_SPI1;
	else if(SPIx == SPI2) rcc = RCC_APB1Periph_SPI2;
	else if(SPIx == SPI3) rcc = RCC_APB1Periph_SPI3;
# ifdef VECTOR_F4
	else if(SPIx == SPI4) rcc = RCC_APB2Periph_SPI4;
	else if(SPIx == SPI5) rcc = RCC_APB2Periph_SPI5;
	else if(SPIx == SPI6) rcc = RCC_APB2Periph_SPI6;
# endif
	return rcc;
}



/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments : 
*
* Reutrn:
*
* Note(s)   : 
*********************************************************************************************************
*/
void bsp_rcc_SPIClockCmd(SPI_TypeDef* SPIx)
{
# ifdef VECTOR_F1
	if(SPIx == SPI1)
		RCC_APB2PeriphClockCmd(bsp_GetSPIRCC(SPIx), ENABLE);
	else if(SPIx == SPI2 || SPIx == SPI3)
		RCC_APB1PeriphClockCmd(bsp_GetSPIRCC(SPIx), ENABLE);
# elif defined VECTOR_F4
	if(SPIx == SPI1 || SPIx == SPI4 || SPIx == SPI5 || SPIx == SPI6)
		RCC_APB2PeriphClockCmd(bsp_GetSPIRCC(SPIx), ENABLE);
	else if(SPIx == SPI2 || SPIx == SPI3)
		RCC_APB1PeriphClockCmd(bsp_GetSPIRCC(SPIx), ENABLE);
# endif
}

/********************************************  END OF FILE  *******************************************/


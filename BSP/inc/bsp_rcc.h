/**
  *******************************************************************************************************
  * File Name: bsp_rcc.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-5-26
  * Brief: 外设时钟管理模块
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-5-26
	*     Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
# ifndef __BSP_RCC_H
# define __BSP_RCC_H

void bsp_rcc_TIMClockCmd(TIM_TypeDef *TIMx);
void bsp_rcc_ADCClockCmd(ADC_TypeDef* ADCx);
void bsp_rcc_USARTClockCmd(USART_TypeDef* USARTx);
void bsp_rcc_DACClcokCmd(DAC_TypeDef* DACx);
void bsp_rcc_SPIClockCmd(SPI_TypeDef* SPIx);
void bsp_rcc_GPIOClcokCmd(GPIO_TypeDef* GPIOx);
void bsp_rcc_DMAClockCmd(DMA_TypeDef* DMAx);
void bsp_rcc_FLASHClockCmd(FLASH_TypeDef* FLASHx);
void bsp_rcc_CRCClockCmd(CRC_TypeDef *CRCx);
void bsp_rcc_EXTIClockCmd(EXTI_TypeDef *EXTIx);
void bsp_rcc_CANClockCmd(CAN_TypeDef* CANx);
void bsp_rcc_WWDGClockCmd(WWDG_TypeDef *WWDGx);
void bsp_rcc_IWDGClockCmd(IWDG_TypeDef *IWDGx);
void bsp_rcc_I2CClockCmd(I2C_TypeDef *I2Cx);


# endif

/********************************************  END OF FILE  *******************************************/


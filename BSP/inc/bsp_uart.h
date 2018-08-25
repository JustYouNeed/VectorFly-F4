/**
  *******************************************************************************************************
  * File Name: bsp_uarth
  * Author: Vector
  * Version: V.0.0
  * Date: 2018-7-29
  * Brief: 本文件提供了飞控底层串口驱动代码
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-7-29
	*			Mod: 建立文件,添加基本函数
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_UART_H
# define __BSP_UART_H

void bsp_uart_Init(void);
bool bsp_uart_GetDataWithTimeout(uint8_t *byte);

void bsp_uart_SendData(uint8_t *buff, uint32_t len);
void bsp_uart_SendDataIsrBlocking(uint8_t *buff, uint32_t len);
int  bsp_uart_PutChar(int c);
void bsp_uart_SendDataDmaBlocking(uint8_t *buff, uint32_t len);

void bsp_uart_Isr(void);
void bsp_uart_DMAIsr(void);

# endif

/********************************************  END OF FILE  *******************************************/

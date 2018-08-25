/**
  *******************************************************************************************************
  * File Name: app_uartlink.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-7-7
  * Brief: 串口链路层
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Data: 2018-7-7
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
# ifndef __APP_UARTLINK_H
# define __APP_UARTLINK_H

void uartlink_Init(void);
void uartlink_SendData(uint8_t *buff, uint32_t len);
bool uartlink_SendPacket(const anoTxPacket_t *packet);

void uartlink_TxTask(void *p_arg);
void uartlink_RxTask(void *p_arg);

# endif
/********************************************  END OF FILE  *******************************************/

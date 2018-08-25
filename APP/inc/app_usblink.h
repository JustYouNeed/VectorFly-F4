/**
  *******************************************************************************************************
  * File Name: app_usblink.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-7-4
  * Brief: 本文件提供对USB虚拟串口的应用层
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-7-4
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
# ifndef __APP_USBLINK_H
# define __APP_USBLINK_H	

void usblink_Init(void);
bool usblink_SendPacket(const anoTxPacket_t *packet);
void usblink_TxTask(void *p_arg);
void usblink_RxTask(void *p_arg);
# endif

/********************************************  END OF FILE  *******************************************/


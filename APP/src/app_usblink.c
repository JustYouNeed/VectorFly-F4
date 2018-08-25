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
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"

# define USBLINK_TX_QUEUE_SIZE		32

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static bool isInit = false;
static xQueueHandle usbTxQueue;
static anoRxPacket_t usbRxPacket;
static RxState_EnumTypeDef usbRxState = WaitForFrameHeader1;
/*
*********************************************************************************************************
*                     usblink_Init                     
*
* Description: USB链路层初始化,创建数据发送消息队列
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void usblink_Init(void)
{
	if(isInit) return;
	
	usbd_cdc_vcp_Init();
	
	usbTxQueue = xQueueCreate(USBLINK_TX_QUEUE_SIZE, sizeof(anoTxPacket_t));
	isInit = true;
}

/*
*********************************************************************************************************
*                     usblink_SendPacket                     
*
* Description: USB链路发送数据包
*             
* Arguments  : 要发送的数据包
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
bool usblink_SendPacket(const anoTxPacket_t *packet)
{
	return xQueueSend(usbTxQueue, packet, 0);
}

/*
*********************************************************************************************************
*                      usblink_RxTask                    
*
* Description: USB链路接收任务
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void usblink_RxTask(void *p_arg)
{
	uint8_t byte = 0;
	uint8_t dataIndex = 0;
	uint8_t checksum = 0;
	usbRxState = WaitForFrameHeader1;
	
	while(1)
	{
		/*  从USB底层接收一个数据包  */
		if(usbGetDataWithTimout(&byte))
		{
			switch(usbRxState)
			{
				case WaitForFrameHeader1:		/*  接收帧头1  */
				{
					usbRxState = (byte == REC_FRAME_HEADER1) ? WaitForFrameHeader2 : WaitForFrameHeader1;
					checksum = byte;
				}break;
				case WaitForFrameHeader2:		/*  帧头2  */
				{
					usbRxState = (byte == REC_FRAME_HEADER2) ? WaitForFunCode : WaitForFrameHeader1;
					checksum += byte;
				}break;
				case WaitForFunCode:		/*  接收功能码  */
				{
					usbRxPacket.FunCode = byte;
					usbRxState = WaitForDataLength;
					checksum += byte;
				}break;
				case WaitForDataLength:		/*  接收数据长度  */
				{
					usbRxPacket.DataLength = byte;
					dataIndex = 0;
					
					/*  如果数据长度为零则说明没有数据需要接收,直接进行数据校验  */
					usbRxState = (byte > 0) ? WaitForData : WaitForCheckSum;
					checksum += byte;
				}break;
				case WaitForData:		/*  接收数据  */
				{
					usbRxPacket.Data[dataIndex] = byte;
					dataIndex ++;
					checksum += byte;
					if(dataIndex == usbRxPacket.DataLength)
						usbRxState = WaitForCheckSum;
				}break;
				case WaitForCheckSum:		/*  数据校验  */
				{
					if(checksum == byte)
					{
						/*  检验成功后就将接收到的数据包发送到ANO模块处理  */
						ano_ReceivePacketBlocking(&usbRxPacket);
					}
					usbRxState = WaitForFrameHeader1;
				}break;
			}
		}
		else		/*  接收超时  */
		{
			usbRxState = WaitForFrameHeader1;
		}
	}
}

/*
*********************************************************************************************************
*                       usblink_TxTask                   
*
* Description: USB发送数据包任务
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void usblink_TxTask(void *p_arg)
{
	anoTxPacket_t usbTxPacket;
	uint8_t sendBuff[32];
	uint8_t checkSum = 0;
	uint8_t dataLen = 0;
	uint8_t i = 0;
	
	while(1)
	{
		/*  从发送消息队列里接收一个要发送的消息  */
		xQueueReceive(usbTxQueue, &usbTxPacket, portMAX_DELAY);
		
		/*  填写相关参数  */
		sendBuff[0] = SEND_FRAME_HEADER1;
		sendBuff[1] = SEND_FRAME_HEADER2;
		sendBuff[2] = usbTxPacket.FunCode;
		sendBuff[3] = usbTxPacket.DataLength;
		memcpy(&sendBuff[4], usbTxPacket.Data, usbTxPacket.DataLength);
		
		/*  计算校验和,因为有四个额外的数据  */
		checkSum = 0;
		for(i = 0; i < usbTxPacket.DataLength + 4; i++)
		{
			checkSum += sendBuff[i];
		}
		dataLen = usbTxPacket.DataLength + 5;
		sendBuff[dataLen - 1] = checkSum;
		
		/*  通过USB底层发送出去  */
		usbsendData(sendBuff, dataLen);
	}
}


/********************************************  END OF FILE  *******************************************/


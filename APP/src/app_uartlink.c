/**
  *******************************************************************************************************
  * File Name: app_uartlink.c
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
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
# define UART_QUEUE_SIZE		16			/*  串口消息队列长度  */
static bool isInit = false;
static xQueueHandle uartTxPacketQueue = NULL;
static anoRxPacket_t uartRxPacket;


/*
*********************************************************************************************************
*                      uartlink_SendData                    
*
* Description: 串口链接对外发送数据函数
*             
* Arguments  : 1.buff: 要发送的数据 缓存区
*							 2.len: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void uartlink_SendData(uint8_t *buff, uint32_t len)
{
	bsp_uart_SendDataDmaBlocking(buff, len);
}

/*
*********************************************************************************************************
*                       uartlink_Init                   
*
* Description: 初始化串口链路层,创建串口通信消息队列
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void uartlink_Init(void)
{
	if(isInit) return;
	
	bsp_uart_Init();
	
	uartTxPacketQueue = xQueueCreate(UART_QUEUE_SIZE, sizeof(anoRxPacket_t));
	
	isInit = true;
}


/*
*********************************************************************************************************
*                        uartlink_SendPacket                  
*
* Description: 通过串口链接发送一个数据包
*             
* Arguments  : 1> packet: 要发送的数据包
*
* Reutrn     : 发送成功或者 失败
*
* Note(s)    : 该函数只是负责将数据包发送到消息队列,如果消息队列已满则会发送失败
*********************************************************************************************************
*/
bool uartlink_SendPacket(const anoTxPacket_t *packet)
{
	return xQueueSend(uartTxPacketQueue, packet, 0);
}


/*
*********************************************************************************************************
*                           uartlink_TxTask               
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
void uartlink_TxTask(void *p_arg)
{
	anoTxPacket_t uartTxPacket;
	uint8_t sendBuff[32];
	uint8_t checkSum = 0;
	uint8_t dataLen = 0;
	uint8_t i = 0;
	
	while(1)
	{
		/*  从发送消息队列里接收一个要发送的消息  */
		xQueueReceive(uartTxPacketQueue, &uartTxPacket, portMAX_DELAY);
		
		/*  填写相关参数  */
		sendBuff[0] = SEND_FRAME_HEADER1;
		sendBuff[1] = SEND_FRAME_HEADER2;
		sendBuff[2] = uartTxPacket.FunCode;
		sendBuff[3] = uartTxPacket.DataLength;
		memcpy(&sendBuff[4], uartTxPacket.Data, uartTxPacket.DataLength);
		
		/*  计算校验和,因为有四个额外的数据  */
		checkSum = 0;
		for(i = 0; i < uartTxPacket.DataLength + 4; i++)
		{
			checkSum += sendBuff[i];
		}
		dataLen = uartTxPacket.DataLength + 5;
		sendBuff[dataLen - 1] = checkSum;
		
		uartlink_SendData(sendBuff, dataLen);
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
void uartlink_RxTask(void *p_arg)
{
	uint8_t byte = 0;
	uint8_t dataIndex = 0;
	uint8_t checksum = 0;
	static RxState_EnumTypeDef uartRxState = WaitForFrameHeader1;
	
	while(1)
	{
		/*  从UART底层接收一个数据包  */
		if(bsp_uart_GetDataWithTimeout(&byte))
		{
			switch(uartRxState)
			{
				case WaitForFrameHeader1:		/*  接收帧头1  */
				{
					uartRxState = (byte == REC_FRAME_HEADER1) ? WaitForFrameHeader2 : WaitForFrameHeader1;
					checksum = byte;
				}break;
				case WaitForFrameHeader2:		/*  帧头2  */
				{
					uartRxState = (byte == REC_FRAME_HEADER2) ? WaitForFunCode : WaitForFrameHeader1;
					checksum += byte;
				}break;
				case WaitForFunCode:		/*  接收功能码  */
				{
					uartRxPacket.FunCode = byte;
					uartRxState = WaitForDataLength;
					checksum += byte;
				}break;
				case WaitForDataLength:		/*  接收数据长度  */
				{
					uartRxPacket.DataLength = byte;
					dataIndex = 0;
					
					/*  如果数据长度为零则说明没有数据需要接收,直接进行数据校验  */
					uartRxState = (byte > 0) ? WaitForData : WaitForCheckSum;
					checksum += byte;
				}break;
				case WaitForData:		/*  接收数据  */
				{
					uartRxPacket.Data[dataIndex] = byte;
					dataIndex ++;
					checksum += byte;
					if(dataIndex == uartRxPacket.DataLength)
						uartRxState = WaitForCheckSum;
				}break;
				case WaitForCheckSum:		/*  数据校验  */
				{
					if(checksum == byte)
					{
						/*  检验成功后就将接收到的数据包发送到ANO模块处理  */
						ano_ReceivePacketBlocking(&uartRxPacket);
					}
					uartRxState = WaitForFrameHeader1;
				}break;
			}
		}
		else		/*  接收超时  */
		{
			uartRxState = WaitForFrameHeader1;
		}
	}
}

/********************************************  END OF FILE  *******************************************/

/**
  *******************************************************************************************************
  * File Name: app_uartlink.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-7-7
  * Brief: ������·��
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Data: 2018-7-7
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

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
# define UART_QUEUE_SIZE		16			/*  ������Ϣ���г���  */
static bool isInit = false;
static xQueueHandle uartTxPacketQueue = NULL;
static anoRxPacket_t uartRxPacket;


/*
*********************************************************************************************************
*                      uartlink_SendData                    
*
* Description: �������Ӷ��ⷢ�����ݺ���
*             
* Arguments  : 1.buff: Ҫ���͵����� ������
*							 2.len: Ҫ���͵����ݳ���
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
* Description: ��ʼ��������·��,��������ͨ����Ϣ����
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
* Description: ͨ���������ӷ���һ�����ݰ�
*             
* Arguments  : 1> packet: Ҫ���͵����ݰ�
*
* Reutrn     : ���ͳɹ����� ʧ��
*
* Note(s)    : �ú���ֻ�Ǹ������ݰ����͵���Ϣ����,�����Ϣ����������ᷢ��ʧ��
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
		/*  �ӷ�����Ϣ���������һ��Ҫ���͵���Ϣ  */
		xQueueReceive(uartTxPacketQueue, &uartTxPacket, portMAX_DELAY);
		
		/*  ��д��ز���  */
		sendBuff[0] = SEND_FRAME_HEADER1;
		sendBuff[1] = SEND_FRAME_HEADER2;
		sendBuff[2] = uartTxPacket.FunCode;
		sendBuff[3] = uartTxPacket.DataLength;
		memcpy(&sendBuff[4], uartTxPacket.Data, uartTxPacket.DataLength);
		
		/*  ����У���,��Ϊ���ĸ����������  */
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
		/*  ��UART�ײ����һ�����ݰ�  */
		if(bsp_uart_GetDataWithTimeout(&byte))
		{
			switch(uartRxState)
			{
				case WaitForFrameHeader1:		/*  ����֡ͷ1  */
				{
					uartRxState = (byte == REC_FRAME_HEADER1) ? WaitForFrameHeader2 : WaitForFrameHeader1;
					checksum = byte;
				}break;
				case WaitForFrameHeader2:		/*  ֡ͷ2  */
				{
					uartRxState = (byte == REC_FRAME_HEADER2) ? WaitForFunCode : WaitForFrameHeader1;
					checksum += byte;
				}break;
				case WaitForFunCode:		/*  ���չ�����  */
				{
					uartRxPacket.FunCode = byte;
					uartRxState = WaitForDataLength;
					checksum += byte;
				}break;
				case WaitForDataLength:		/*  �������ݳ���  */
				{
					uartRxPacket.DataLength = byte;
					dataIndex = 0;
					
					/*  ������ݳ���Ϊ����˵��û��������Ҫ����,ֱ�ӽ�������У��  */
					uartRxState = (byte > 0) ? WaitForData : WaitForCheckSum;
					checksum += byte;
				}break;
				case WaitForData:		/*  ��������  */
				{
					uartRxPacket.Data[dataIndex] = byte;
					dataIndex ++;
					checksum += byte;
					if(dataIndex == uartRxPacket.DataLength)
						uartRxState = WaitForCheckSum;
				}break;
				case WaitForCheckSum:		/*  ����У��  */
				{
					if(checksum == byte)
					{
						/*  ����ɹ���ͽ����յ������ݰ����͵�ANOģ�鴦��  */
						ano_ReceivePacketBlocking(&uartRxPacket);
					}
					uartRxState = WaitForFrameHeader1;
				}break;
			}
		}
		else		/*  ���ճ�ʱ  */
		{
			uartRxState = WaitForFrameHeader1;
		}
	}
}

/********************************************  END OF FILE  *******************************************/

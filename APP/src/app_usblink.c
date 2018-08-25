/**
  *******************************************************************************************************
  * File Name: app_usblink.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-7-4
  * Brief: ���ļ��ṩ��USB���⴮�ڵ�Ӧ�ò�
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-7-4
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
* Description: USB��·���ʼ��,�������ݷ�����Ϣ����
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
* Description: USB��·�������ݰ�
*             
* Arguments  : Ҫ���͵����ݰ�
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
* Description: USB��·��������
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
		/*  ��USB�ײ����һ�����ݰ�  */
		if(usbGetDataWithTimout(&byte))
		{
			switch(usbRxState)
			{
				case WaitForFrameHeader1:		/*  ����֡ͷ1  */
				{
					usbRxState = (byte == REC_FRAME_HEADER1) ? WaitForFrameHeader2 : WaitForFrameHeader1;
					checksum = byte;
				}break;
				case WaitForFrameHeader2:		/*  ֡ͷ2  */
				{
					usbRxState = (byte == REC_FRAME_HEADER2) ? WaitForFunCode : WaitForFrameHeader1;
					checksum += byte;
				}break;
				case WaitForFunCode:		/*  ���չ�����  */
				{
					usbRxPacket.FunCode = byte;
					usbRxState = WaitForDataLength;
					checksum += byte;
				}break;
				case WaitForDataLength:		/*  �������ݳ���  */
				{
					usbRxPacket.DataLength = byte;
					dataIndex = 0;
					
					/*  ������ݳ���Ϊ����˵��û��������Ҫ����,ֱ�ӽ�������У��  */
					usbRxState = (byte > 0) ? WaitForData : WaitForCheckSum;
					checksum += byte;
				}break;
				case WaitForData:		/*  ��������  */
				{
					usbRxPacket.Data[dataIndex] = byte;
					dataIndex ++;
					checksum += byte;
					if(dataIndex == usbRxPacket.DataLength)
						usbRxState = WaitForCheckSum;
				}break;
				case WaitForCheckSum:		/*  ����У��  */
				{
					if(checksum == byte)
					{
						/*  ����ɹ���ͽ����յ������ݰ����͵�ANOģ�鴦��  */
						ano_ReceivePacketBlocking(&usbRxPacket);
					}
					usbRxState = WaitForFrameHeader1;
				}break;
			}
		}
		else		/*  ���ճ�ʱ  */
		{
			usbRxState = WaitForFrameHeader1;
		}
	}
}

/*
*********************************************************************************************************
*                       usblink_TxTask                   
*
* Description: USB�������ݰ�����
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
		/*  �ӷ�����Ϣ���������һ��Ҫ���͵���Ϣ  */
		xQueueReceive(usbTxQueue, &usbTxPacket, portMAX_DELAY);
		
		/*  ��д��ز���  */
		sendBuff[0] = SEND_FRAME_HEADER1;
		sendBuff[1] = SEND_FRAME_HEADER2;
		sendBuff[2] = usbTxPacket.FunCode;
		sendBuff[3] = usbTxPacket.DataLength;
		memcpy(&sendBuff[4], usbTxPacket.Data, usbTxPacket.DataLength);
		
		/*  ����У���,��Ϊ���ĸ����������  */
		checkSum = 0;
		for(i = 0; i < usbTxPacket.DataLength + 4; i++)
		{
			checkSum += sendBuff[i];
		}
		dataLen = usbTxPacket.DataLength + 5;
		sendBuff[dataLen - 1] = checkSum;
		
		/*  ͨ��USB�ײ㷢�ͳ�ȥ  */
		usbsendData(sendBuff, dataLen);
	}
}


/********************************************  END OF FILE  *******************************************/


/**
  *******************************************************************************************************
  * File Name: app_ano.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: �������ģ��,�ṩ����λ��ͨ�ŵĹ���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�,��Ӧ������λ��V5.0
	*		
	*
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"


# define	PROTO_VERSION		5.0


//PID�������ϴ�������ʱʹ�õı任����
# define ANO_PID_TRAN_FAC_P 10.0f
# define ANO_PID_TRAN_FAC_I 10.0f
# define ANO_PID_TRAN_FAC_D	1.0f


/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
bool isInit = false;
static xQueueHandle anoRxQueue = NULL;


/*
*********************************************************************************************************
*                      debug_Response                    
*
* Description: ��Ӧ��λ��������
*             
* Arguments  : 1> funcode: Ҫ��Ӧ�Ĺ�����
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void ano_Response(void)
{
//	uint8_t SendBuff[8] = {0};
//	uint8_t i;
//	
//	SendBuff[0] = 0XAA;//֡��ʼ
//	SendBuff[1] = 0XAA;
//	SendBuff[2] = DTH_CHECK;
//	SendBuff[3] = 7;
//	SendBuff[4] = ANO_RX.FunCode;
//	SendBuff[5] = ANO_RX.Data[ANO_RX.DataLength]&0xff;
//	
//	for(i = 0; i < 6; i++) SendBuff[6] += SendBuff[i];
//	
//	ano_SendData(SendBuff, 7);
}

/*
*********************************************************************************************************
*                     app_ano_DataUpload                     
*
* Description: ���Թ��������ϴ�
*             
* Arguments  : 1> buff: ���ݻ�����
*              2> funcode: ������
*							 3> len: ���ݳ���
*
* Reutrn     : 1> 0: ����ִ�гɹ�
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t ano_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32] = {0};
	uint8_t i;
	if(len>27)return 0;//���ݳ��ȳ�������
	if(funCode>0xff) return 1;//���������
		
	SendBuff[0] = 0XAA; //֡��ʼ
	SendBuff[1] = 0XAA; //֡��ʼ
	SendBuff[2] = funCode;  //������
	SendBuff[3] = len;   //���ݳ��ȣ���ȥ��ʼ��͹������Լ�����
	
	/*  �����ݸ��Ƶ�������������У���  */
	for(i = 0; i < len; i++) 
	{
		SendBuff[i + 4] = buff[i];
		SendBuff[len + 4] += SendBuff[i];
	}
	/*  ����ʣ���У��  */
	for(i = len; i< len + 4; i++) SendBuff[len + 4] += SendBuff[i];
	
//	ano_SendData(SendBuff, len + 5);
	
	return 0;
}



/*
*********************************************************************************************************
*                            app_ano_HandlerAck              
*
* Description: ����ACK����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void ano_ACKHandler(anoRxPacket_t packet)
{
	uint8_t ack = packet.Data[0];

	switch(ack)
	{
		case ACK_PID: break;		/*  ����PID  */
		case ACK_FLY_MODE: break;
		case ACK_W_OFFSET: break;
		case ACK_FP_NUM: break; 
		case ACK_LOCAL_MODE: break;
		case ACK_TRANS_MODE:break;
		case ACK_OF_MODE: break;
		case ACK_VERSION: break;
		case ACK_RESET_PID: break;
		case ACK_RESET_PARA: break;
		default: break;
	}
}
/*
*********************************************************************************************************
*                      app_ano_HandleCommand                    
*
* Description: ������λ���·���COMMANDָ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void ano_CMDHandler(anoRxPacket_t packet)
{
	uint8_t cmd = packet.Data[0];

	switch(cmd)
	{
		case CMD_CAL_ACC: break;
		case CMD_CAL_GYRO: break;
		case CMD_CAL_ALT: break; 
		case CMD_CAL_MAG: break;
		case CMD_CAL_BARO:break;
		case CMD_CAL_INE: break;
		case CMD_EXIT_CAL: break;
		case CMD_EXIT_CAL_1: break;
		case CMD_EXIT_CAL_2: break;
		case CMD_EXIT_CAL_3: break;
		case CMD_EXIT_CAL_4:break;
		case CMD_EXIT_CAL_5: break;
		case CMD_EXIT_CAL_6: break;
		case CMD_LOCK: break;
		case CMD_UNLOCK: break;
		default: break;
	}
}
/*
*********************************************************************************************************
*                     ano_PacketHandler                     
*
* Description: �Խ��յ������ݰ����н���
*             
* Arguments  : 1> packet: Ҫ���������ݰ�
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void ano_PacketAnalysis(anoRxPacket_t packet)
{
	switch(packet.FunCode)
	{
		case HTD_COMMAND: ano_CMDHandler(packet);break;
		case HTD_ACK: ano_ACKHandler(packet);break;
		case HTD_RCDATA:break;
		case HTD_READ_PARA:break;
		case HTD_SET_PARA:break;
		case HTD_FLY_MODE:break;
		case HTD_PID1: ano_Response(); break;
		case HTD_PID2: ano_Response(); break;
		case HTD_PID3: ano_Response(); break;
		case HTD_PID4: ano_Response(); break;
		case HTD_PID5: ano_Response(); break;
		case HTD_PID6: ano_Response(); break;
		case HTD_READ_FP: break;
		case HTD_SET_FP: break;
		case HTD_SET_LOCAL: break;
		case HTD_SET_LOCAL_2: break;
		case HTD_SET_RADIOLINK: break;
		case HTD_SET_OPT_FLOW: break;
		case HTD_OPT_FLOW_CMD: break;
		case HTD_IAP: break;
		default:break;
	}
}


/*
*********************************************************************************************************
*                       ano_ReceivePacketBlocking                   
*
* Description: ANO����һ�����ݰ�,��һ�����ݰ����͵�ANO�Ľ�����Ϣ����
*             
* Arguments  : ANO���ݰ�
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
bool ano_ReceivePacketBlocking(anoRxPacket_t *packet)
{
	return xQueueSend(anoRxQueue, packet, portMAX_DELAY);
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
void ano_Init(void)
{
	if(isInit) return;
	
	/*  �������ݽ�����Ϣ����  */
	anoRxQueue = xQueueCreate(10, sizeof(anoRxPacket_t));
	isInit = true;
}

/*
*********************************************************************************************************
*                       ano_PacketRxTask                   
*
* Description: ANO�������ݰ�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void ano_PacketRxTask(void *p_arg)
{
	anoRxPacket_t anoRxPacket;
	
	while(1)
	{
		/*  ����Ϣ�����������һ�����ݰ�  */
		xQueueReceive(anoRxQueue, &anoRxPacket, portMAX_DELAY);
		
		/*  �����յ������ݰ����д���  */
		ano_PacketAnalysis(anoRxPacket);
	}
}
/********************************************  END OF FILE  *******************************************/


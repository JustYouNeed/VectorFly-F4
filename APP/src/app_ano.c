/**
  *******************************************************************************************************
  * File Name: app_ano.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 程序调试模块,提供与上位机通信的功能
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件,对应匿名上位机V5.0
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


//PID参数在上传及接收时使用的变换因子
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
* Description: 响应上位机的命令
*             
* Arguments  : 1> funcode: 要响应的功能码
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
//	SendBuff[0] = 0XAA;//帧起始
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
* Description: 调试功能数据上传
*             
* Arguments  : 1> buff: 数据缓存区
*              2> funcode: 功能码
*							 3> len: 数据长度
*
* Reutrn     : 1> 0: 函数执行成功
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t ano_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32] = {0};
	uint8_t i;
	if(len>27)return 0;//数据长度超过限制
	if(funCode>0xff) return 1;//功能码错误
		
	SendBuff[0] = 0XAA; //帧起始
	SendBuff[1] = 0XAA; //帧起始
	SendBuff[2] = funCode;  //功能码
	SendBuff[3] = len;   //数据长度，除去起始码和功能码以及长度
	
	/*  将数据复制到发送区并计算校验和  */
	for(i = 0; i < len; i++) 
	{
		SendBuff[i + 4] = buff[i];
		SendBuff[len + 4] += SendBuff[i];
	}
	/*  计算剩余的校验  */
	for(i = len; i< len + 4; i++) SendBuff[len + 4] += SendBuff[i];
	
//	ano_SendData(SendBuff, len + 5);
	
	return 0;
}



/*
*********************************************************************************************************
*                            app_ano_HandlerAck              
*
* Description: 处理ACK命令
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
		case ACK_PID: break;		/*  请求PID  */
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
* Description: 处理上位机下发的COMMAND指令
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
* Description: 对接收到的数据包进行解析
*             
* Arguments  : 1> packet: 要解析的数据包
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
* Description: ANO接收一个数据包,将一个数据包发送到ANO的接收消息队列
*             
* Arguments  : ANO数据包
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
	
	/*  创建数据接收消息队列  */
	anoRxQueue = xQueueCreate(10, sizeof(anoRxPacket_t));
	isInit = true;
}

/*
*********************************************************************************************************
*                       ano_PacketRxTask                   
*
* Description: ANO接收数据包任务
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
		/*  从消息队列里面接收一个数据包  */
		xQueueReceive(anoRxQueue, &anoRxPacket, portMAX_DELAY);
		
		/*  将接收到的数据包进行处理  */
		ano_PacketAnalysis(anoRxPacket);
	}
}
/********************************************  END OF FILE  *******************************************/


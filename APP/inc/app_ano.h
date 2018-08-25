/**
  *******************************************************************************************************
  * File Name: app_debug.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-16
  * Brief: ���ļ��������й�ʹ�ô�����λ��������λ�������ĺ������������궨��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-16
	*     Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
# ifndef __APP_ANO_H
# define __APP_ANO_H


# define AS_HOST		0u
# define AS_DEVICE	1u

# define FREME_LEN			32

# if AS_DEVICE > 0u
	/*  ֡ͷ  */
	# define REC_FRAME_HEADER1	0XAA
	# define REC_FRAME_HEADER2	0XAF
	
	# define SEND_FRAME_HEADER1	0XAA
	# define SEND_FRAME_HEADER2	0XAA
# else
		/*  ֡ͷ  */
	# define REC_FRAME_HEADER1	0XAA
	# define REC_FRAME_HEADER2	0XAA
	
	# define SEND_FRAME_HEADER1	0XAA
	# define SEND_FRAME_HEADER2	0XAF
# endif


/*  ���ǻ����͸�ģ�������,HTD: Host to Device  */
typedef enum
{
	HTD_COMMAND = 0x01,						/*  ����  */
	HTD_ACK = 0x02,								/*  ��ȡ����  */
	HTD_RCDATA = 0x03,						/*  ���п�������(������΢�ͷɻ�)  */
	HTD_READ_PARA = 0x08,					/*  ��ȡ����  */
	HTD_SET_PARA = 0x09,					/*  ��������  */
	HTD_FLY_MODE = 0x0a,					/*  ����ģʽ  */
	HTD_PID1 = 0x10,							/*  ����PID����1  */
	HTD_PID2 = 0x11,
	HTD_PID3 = 0x12,
	HTD_PID4 = 0x13,
	HTD_PID5 = 0x14,
	HTD_PID6 = 0x15,
	HTD_READ_FP = 0x20,						/*  ��ȡ��CNT��������Ϣ  */
	HTD_SET_FP = 0x21,						/*  д�뺽����Ϣ  */
	HTD_SET_LOCAL = 0x3a,					
	HTD_SET_LOCAL_2 = 0x3b,
	HTD_SET_RADIOLINK = 0x40,
	HTD_SET_OPT_FLOW = 0x50,
	HTD_OPT_FLOW_CMD = 0x56,			/*  ��������  */
	HTD_IAP = 0xf0,								/*  ����IAP  */
}HTD_FunCode_EnumTypeDef;

/*  ���յ�COMMAND������������  */
typedef enum
{
	CMD_CAL_ACC = 0x01,			/*  У׼ACC  */
	CMD_CAL_GYRO = 0x02,		/*  У׼GYRO  */
	CMD_CAL_ALT = 0x03,			/*  У׼ALT�߶�  */
	CMD_CAL_MAG = 0x04,			/*  У׼MAG  */
	CMD_CAL_BARO = 0x05,		/*  У׼BARO  */
	CMD_CAL_INE = 0x10,			/*  ����У׼  */
	
	CMD_EXIT_CAL = 0x20,		/*  �˳�6��У׼  */
	CMD_EXIT_CAL_1 = 0x21,
	CMD_EXIT_CAL_2 = 0x22,
	CMD_EXIT_CAL_3 = 0x23,
	CMD_EXIT_CAL_4 = 0x24,
	CMD_EXIT_CAL_5 = 0x25,
	CMD_EXIT_CAL_6 = 0x26,
	
	CMD_LOCK = 0xa0,				/*  �ɿ�����(�������ֻ���������)  */
	CMD_UNLOCK = 0xa1,			/*  �ɿؽ���(�������ֻ���������)  */
}COMMAND_FunCode_EnumTypeDef;

/*  ACK�����������  */
typedef enum
{
	ACK_PID = 0x01,
	ACK_FLY_MODE = 0x02,
	ACK_W_OFFSET = 0x0c,
	ACK_FP_NUM = 0x21,
	ACK_LOCAL_MODE = 0x30,
	ACK_TRANS_MODE = 0x40,
	ACK_OF_MODE = 0x50,
	ACK_VERSION = 0xa0,
	ACK_RESET_PID = 0xa1,
	ACK_RESET_PARA = 0xa2,
}ACK_FunCode_EnumTypeDef;

/*  ģ�鷢�͸���λ��������  */
typedef enum
{
	DTH_VERSION = 0x00,					/*  �汾��Ϣ  */
	DTH_STATUS = 0x01,					/*  �ɻ���̬�Ȼ�����Ϣ  */
	DTH_SENSOR = 0x02,					/*  �ɻ�����������  */
	DTH_RECDATA = 0x03,					/*  �ɻ����յ��Ŀ�������  */
	DTH_GPSDATA = 0x04,					/*  ����GPS����  */
	DTH_POWER = 0x05,						/*  ����  */
	DTH_MOTO = 0x06,						/*  ���PWM  */
	DTH_SENSOR2 = 0x07,					
	DTH_PARA_SET = 0x09,
	DTH_FLY_MODE = 0x0a,
	DTH_PID1 = 0x10,
	DTH_PID2 = 0x11,
	DTH_PID3 = 0x12,
	DTH_PID4 = 0x13,
	DTH_PID5 = 0x14,
	DTH_PID6 = 0x15,
	DTH_FP_NUM = 0x20,
	DTH_FP = 0x21,
	DTH_DISTANCE = 0x30,
	DTH_LOCAL = 0x32,
	DTH_SET_LOCAL = 0x3a,
	DTH_SET_LOCAL2 = 0x3b,
	DTH_SET_RADIOLINK = 0x40,
	DTH_SET_OF = 0x50,
	DTH_OF_DATA1 = 0x51,
	DTH_OF_DATA2 = 0x52,
	DTH_OF_DATA3 = 0x53,
	DTH_OF_DATA4 = 0x54,
	DTH_OF_DATA5 = 0x55,
	DTH_STRING = 0xe0,
	DTH_MSG = 0xee,
	DTH_CHECK = 0xef,
	DTH_USER1 = 0xf1,					/*  �û�����1  */
	DTH_USER2 = 0xf2,
	DTH_USER3 = 0xf3,
	DTH_USER4 = 0xf4,
	DTH_USER5 = 0xf5,
	DTH_USER6 = 0xf6,
	DTH_USER7 = 0xf7,
	DTH_USER8 = 0xf8,
	DTH_USER9 = 0xf9,
	DTH_USER10 = 0xfa,
}DTH_FunCode_EnumTypeDef;

/*  ����֡�ṹ��  */
typedef struct
{
	uint8_t Device;
	uint8_t SrcAddr;
	uint8_t DestAddr;
	uint8_t FunCode;
	uint8_t DataLength;
	uint8_t Data[FREME_LEN];
	uint8_t CheckSum;
}anoRxPacket_t;

/*  ANO�������ݰ�  */
typedef struct 
{
	uint8_t FunCode;
	uint8_t DataLength;
	uint8_t Data[FREME_LEN];
}anoTxPacket_t;

/*  ���ݽ���״̬  */
typedef enum
{
	WaitForFrameHeader1,
	WaitForFrameHeader2,
	WaitForFunCode,
	WaitForDataLength,
	WaitForData,
	WaitForCheckSum,
}RxState_EnumTypeDef;



void ano_PacketAnalysis(const anoRxPacket_t packet);
void ano_Init(void);
void ano_ReceiveData(uint8_t data);

void ano_PacketRxTask(void *p_arg);
bool ano_ReceivePacketBlocking(anoRxPacket_t *packet);
# endif

/********************************************  END OF FILE  *******************************************/
	


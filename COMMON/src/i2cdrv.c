/**
  *******************************************************************************************************
  * File Name: i2cdrv.c
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-3-4
  * Brief: ���ļ��ṩ���й�i2cͨ�ŵĵײ�����,���ģ��,��оƬ�޹�,����������,ʹ��ʱ��ҪΪÿһ��i2cͨ��
	*				 ��������һ��i2cDevice�ṹ��,���ڿ���ͨ��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �����ļ�,��ɻ�������
	*
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: �޸���ʱ����Ϊ��,ʹ������������ֲ
	*
	*		3.Author: Vector
	*			Date: 2018-8-23
	*			Mod: �޸�IIC�����߼�,�޸�ԭ��IIC���ƽṹ��,��������ֲ
	*
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"
# include "i2cdrv.h"

# define i2c_DelayMs(x)		bsp_tim_DelayMs(x)
# define i2c_Delayus(x)		bsp_tim_DelayUs(x)

/*
*********************************************************************************************************
*                           i2cdrv_SetSDAHigh               
*
* Description: ����SDA��Ϊ�ߵ�ƽ
*             
* Arguments  : i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_SetSDAHigh(const i2cDevice i2c)
{
	PIN_OUT_HIGH(i2c.sdaPort, i2c.sdaPin);
}

/*
*********************************************************************************************************
*                           i2cdrv_SetSDALow               
*
* Description: ����SDA��Ϊ�͵�ƽ
*             
* Arguments  : i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_SetSDALow(const i2cDevice i2c)
{
	PIN_OUT_LOW(i2c.sdaPort, i2c.sdaPin);
}
/*
*********************************************************************************************************
*                             i2cdrv_SetSDAIn             
*
* Description: ����i2cͨ��SDA����Ϊ����ģʽ 
*             
* Arguments  : i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : ���ļ�˽�к���
*********************************************************************************************************
*/
static void i2cdrv_SetSDAIn(const i2cDevice i2c)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = i2c.sdaPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
# ifdef VECTOR_F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
# else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
# endif
	GPIO_Init(i2c.sdaPort, &GPIO_InitStructure);//��ʼ��
}

/*
*********************************************************************************************************
*                             i2cdrv_SetSDAOut             
*
* Description: ����i2cͨ��SDA����Ϊ���ģʽ
*             
* Arguments  : i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : ���ļ�˽�к���
*********************************************************************************************************
*/
static void i2cdrv_SetSDAOut(const i2cDevice i2c)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = i2c.sdaPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
# ifdef VECTOR_F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
# else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP;
# endif
	GPIO_Init(i2c.sdaPort, &GPIO_InitStructure);//��ʼ��
}



/*
*********************************************************************************************************
*                           i2cdrv_SetSCLHigh               
*
* Description: ����SCL��Ϊ�ߵ�ƽ
*             
* Arguments  : i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_SetSCLHigh(const i2cDevice i2c)
{
	PIN_OUT_HIGH(i2c.sclPort, i2c.sclPin);
}

/*
*********************************************************************************************************
*                           i2cdrv_SetSCLLow               
*
* Description: ����SCL��Ϊ�͵�ƽ
*             
* Arguments  : i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_SetSCLLow(const i2cDevice i2c)
{
	PIN_OUT_LOW(i2c.sclPort, i2c.sclPin);
}

/*
*********************************************************************************************************
*                        i2cdrv_ReadSDA                 
*
* Description: ��ȡSDA�ź���
*             
* Arguments  : 1.i2c: i2cͨ�ſ��ƽṹ��
*
* Reutrn     : SDA�ź���״̬
*
* Note(s)    : None.
*********************************************************************************************************
*/
static uint8_t i2cdrv_ReadSDA(const i2cDevice i2c)
{
	return PIN_READ(i2c.sdaPort, i2c.sdaPin);
}


/*
*********************************************************************************************************
*                               i2cdrv_Start           
*
* Description: i2cͨ�������ź�
*             
* Arguments  : 1.i2c: i2cͨ�ſ��ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_Start(i2cDevice i2c)
{
	i2cdrv_SetSDAOut(i2c);
	
	i2cdrv_SetSDAHigh(i2c);
	i2cdrv_SetSCLHigh(i2c);
	
	i2c_Delayus(2);
	i2cdrv_SetSDALow(i2c);
	i2c_Delayus(2);
	i2cdrv_SetSCLLow(i2c);
}

/*
*********************************************************************************************************
*                     i2cdrv_Stop                     
*
* Description: ����������ֹͣi2cͨ��ָ��
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_Stop(i2cDevice i2c)
{
	i2cdrv_SetSDAOut(i2c);
	i2cdrv_SetSCLLow(i2c);
	i2cdrv_SetSDALow(i2c);
	
	i2c_Delayus(2);
	
	i2cdrv_SetSCLHigh(i2c);
	i2cdrv_SetSDAHigh(i2c);
	
	i2c_Delayus(2);
}

/*
*********************************************************************************************************
*                    i2cdrv_Ack                      
*
* Description: ������������Ӧ����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_Ack(i2cDevice i2c)
{
	i2cdrv_SetSCLLow(i2c);
	
	i2cdrv_SetSDAOut(i2c);
	
	i2cdrv_SetSDALow(i2c);
	
	i2c_Delayus(2);
	
	i2cdrv_SetSCLHigh(i2c);
	
	i2c_Delayus(2);
	
	i2cdrv_SetSCLLow(i2c);
}

/*
*********************************************************************************************************
*                      i2cdrv_NoAck                    
*
* Description: ��������������Ӧ����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_NoAck(i2cDevice i2c)
{
	i2cdrv_SetSCLLow(i2c);
	i2cdrv_SetSDAOut(i2c);
	i2cdrv_SetSDAHigh(i2c);
	
	i2c_Delayus(2);
	
	i2cdrv_SetSCLHigh(i2c);
	
	i2c_Delayus(2);
	
	i2cdrv_SetSCLLow(i2c);
}

/*
*********************************************************************************************************
*                         i2cdrv_WaitAck                 
*
* Description: �ȴ�������Ӧ����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*
* Reutrn     : ����1����Ӧ,0����Ӧ
*
* Note(s)    : None.
*********************************************************************************************************
*/
static uint8_t i2cdrv_WaitAck(i2cDevice i2c)
{
	uint8_t ErrTime = 0;

	i2cdrv_SetSDAHigh(i2c);
	i2c_Delayus(1);
	i2cdrv_SetSCLHigh(i2c);
	i2c_Delayus(1);

	i2cdrv_SetSDAIn(i2c);
	while(i2cdrv_ReadSDA(i2c))
	{
		ErrTime ++;
		if(ErrTime > 250)
		{
			i2cdrv_Stop(i2c);
			return 1;
		}
	}
	i2cdrv_SetSCLLow(i2c);
	return 0;
}

/*
*********************************************************************************************************
*                         i2cdrv_SendByte                 
*
* Description: ͨ��i2c����һ���ֽڵ�����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.byte: Ҫ���͵�����,8λ����
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void i2cdrv_SendByte(i2cDevice i2c, uint8_t byte)
{
	uint8_t cnt = 0x0;
	
	i2cdrv_SetSDAOut(i2c);
	
	i2cdrv_SetSCLLow(i2c);
	
	for(; cnt < 8; cnt ++)
	{
		if((byte&0x80)>>7 == 1) i2cdrv_SetSDAHigh(i2c);
		else if((byte&0x80)>>7 == 0) i2cdrv_SetSDALow(i2c);
		byte <<= 1;
		
		i2c_Delayus(1);
		i2cdrv_SetSCLHigh(i2c);
		i2c_Delayus(1);
		i2cdrv_SetSCLLow(i2c);
		i2c_Delayus(1);
	}
}

/*
*********************************************************************************************************
*                           i2cdrv_ReadByte               
*
* Description: i2c��ȡһ���ֽڵ�����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.ack: �Ƿ���Ҫ����ack, 0,����Ҫ,1��Ҫ
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static uint8_t i2cdrv_ReadByte(i2cDevice i2c, uint8_t ack)
{
	uint8_t cnt = 0x0, rec = 0x0;
	
	i2cdrv_SetSDAIn(i2c);
	
	for(; cnt < 8; cnt ++)
	{
		i2cdrv_SetSCLLow(i2c);
		i2c_Delayus(1);
		i2cdrv_SetSCLHigh(i2c);
		
		rec <<= 1;
		if(i2cdrv_ReadSDA(i2c) == 1) rec ++;
		
		i2c_Delayus(1);
	}
	if(!ack)
		i2cdrv_NoAck(i2c);
	else
		i2cdrv_Ack(i2c);
	return rec;
}


/*
*********************************************************************************************************
*                          i2c_WriteByte                
*
* Description: ��i2c�豸�ĵ�ַдһ���ֽڵ�����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.byte: Ҫд�������
*
* Reutrn     : д��ɹ�����ʧ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_WriteByte(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t byte)
{
	return i2c_WriteBuff(i2c, devAddr, memAddr, 1, &byte);
}

/*
*********************************************************************************************************
*                          i2c_WriteBuff                
*
* Description: ��i2c�豸�ĵ�ַд����ֽڵ�����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.len: Ҫд������ݳ���
*							 5.buff: Ҫд������ݻ�������ַ
*
* Reutrn     : д��ɹ�����ʧ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_WriteBuff(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t len, uint8_t *buff)
{
	uint8_t cnt = 0;
	i2cdrv_Start(i2c);
	i2cdrv_SendByte(i2c, (devAddr ) | 0);
	if(i2cdrv_WaitAck(i2c))
	{
		i2cdrv_Stop(i2c);
		return 1;
	}
	i2cdrv_SendByte(i2c, memAddr);
	i2cdrv_WaitAck(i2c);
	
	for(cnt = 0; cnt < len; cnt ++)
	{
		i2cdrv_SendByte(i2c, buff[cnt]);
		if(i2cdrv_WaitAck(i2c))
		{
			i2cdrv_Stop(i2c);
			return 1;
		}
	}
	
	return 0;
}


/*
*********************************************************************************************************
*                       i2c_ReadByte                   
*
* Description: ��i2c�豸��ĳһ����ַ��ȡһ���ֽڵ�����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.byte: ���ݴ洢��ַ
*
* Reutrn     : ��ȡ�ɹ�����ʧ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t *byte)
{
	return i2c_ReadBuff(i2c, devAddr, memAddr, 1, byte);
}


/*
*********************************************************************************************************
*                      i2c_ReadBuff                    
*
* Description: ��i2c�豸��ĳһ����ַ�ж�ȡ����ֽڵ�����
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.len: Ҫ��ȡ�����ݳ���
*							 5.buff: ���ݴ洢��ַ
*
* Reutrn     : ��ȡ�ɹ�����ʧ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_ReadBuff(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t len, uint8_t *buff)
{
	i2cdrv_Start(i2c);
	i2cdrv_SendByte(i2c, (devAddr ) | 0);
	if(i2cdrv_WaitAck(i2c))
	{
		i2cdrv_Stop(i2c);
		return 1;
	}
	i2cdrv_SendByte(i2c, memAddr);
	i2cdrv_WaitAck(i2c);
	i2cdrv_Start(i2c);
	i2cdrv_SendByte(i2c, (devAddr ) | 1);
	i2cdrv_WaitAck(i2c);
	
	while(len)
	{
		if(len == 1) *buff = i2cdrv_ReadByte(i2c, 0);
		else *buff = i2cdrv_ReadByte(i2c, 1);
		
		len --;
		buff ++;
	}
	i2cdrv_Stop(i2c);
	return 0;
}


/*
*********************************************************************************************************
*                       i2c_WriteBit                   
*
* Description: дi2c�豸��ĳһ���Ĵ�����ĳһλ
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.bitx: Ҫд�ļĴ���λ
*							 5.data: д0����д1
*
* Reutrn     : д���ļĴ���ֵ
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_WriteBit(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitx, uint8_t data)
{
	uint8_t byte = 0;
	
	/*  ��Ҫ�ȴӼĴ����ж�ȡԭ����ֵ,��ֹӰ������λ  */
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);
	
	byte = (data == 0) ? (byte & ~(1 << bitx)) : (byte | (1 << bitx));
	return i2c_WriteByte(i2c, devAddr, memAddr, byte);
}

/*
*********************************************************************************************************
*                      i2c_WriteBits                    
*
* Description: дi2c�豸��ĳһ���Ĵ����Ķ��λ
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.bitStart: Ҫд�ļĴ�����ʼλ
*							 5.bitLen: Ҫд��λ����
*							 6.data: Ҫд�������
*
* Reutrn     : д���ļĴ���ֵ
*
* Note(s)    : �ú���ֻ��д�����ļĴ���λ,����д��������λ
*********************************************************************************************************
*/
uint8_t i2c_WriteBits(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitStart, uint8_t bitLen, uint8_t data)
{
	uint8_t byte = 0;
	
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);
	if(byte != 0xff)
	{
		uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
		data <<= (bitStart - bitLen + 1);	/*  �����ݲ��뵽�����λ��  */
		data &= mask;		/* 	��������λ  */
		byte &= ~mask;	/*  ���ԭ����λ  */
		byte |= data;		/*  �ϲ�����  */
		i2c_WriteByte(i2c, devAddr, memAddr, byte);
	}
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);	/*  ��ȡ���ú�ļĴ���ֵ  */
	return byte;
}


/*
*********************************************************************************************************
*                       i2c_ReadBit                   
*
* Description: ��ȡi2c�豸��ĳһ���Ĵ�����ĳһλ
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.bitx: Ҫ��ȡ��λ
*						   5.data: ���ڴ洢����Ļ�������ַ
*
* Reutrn     : ��ȡ�ɹ�����ʧ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_ReadBit(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitx, uint8_t *data)
{
	uint8_t byte = 0;
	
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);
	
	return ((byte >> bitx) & 0x01);
}


/*
*********************************************************************************************************
*                       i2c_ReadBits                   
*
* Description: ��i2c�豸�ļĴ����Ķ��λ
*             
* Arguments  : 1.i2c: i2c���ƽṹ��
*							 2.devAddr: �豸��ַ
*							 3.memAddr: �ڴ��ַ
*							 4.bitStart: Ҫ��ȡ�Ŀ�ʼλ
*							 5.bitLen: Ҫ��ȡ��λ����
*							 6.data: ���ڴ洢����Ļ�������ַ
*
* Reutrn     : ��ȡ�ɹ�����ʧ��
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_ReadBits(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitStart, uint8_t bitLen, uint8_t *data)
{
	bool status;
	uint8_t byte;

	status = i2c_ReadByte(i2c, devAddr, memAddr, &byte);
	
	uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
	byte &= mask;
	byte >>= (bitStart - bitLen + 1);
	*data = byte;
	
	return status;
}



/********************************************  END OF FILE  *******************************************/



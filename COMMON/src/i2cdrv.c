/**
  *******************************************************************************************************
  * File Name: i2cdrv.c
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-3-4
  * Brief: 本文件提供了有关i2c通信的底层驱动,软件模拟,与芯片无关,函数可重入,使用时需要为每一个i2c通信
	*				 单独声明一个i2cDevice结构体,用于控制通信
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: 建立文件,完成基本函数
	*
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: 修改延时函数为宏,使驱动更容易移植
	*
	*		3.Author: Vector
	*			Date: 2018-8-23
	*			Mod: 修改IIC控制逻辑,修改原来IIC控制结构体,更容易移植
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
* Description: 设置SDA脚为高电平
*             
* Arguments  : i2c控制结构体
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
* Description: 设置SDA脚为低电平
*             
* Arguments  : i2c控制结构体
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
* Description: 设置i2c通信SDA引脚为输入模式 
*             
* Arguments  : i2c控制结构体
*
* Reutrn     : None.
*
* Note(s)    : 本文件私有函数
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
	GPIO_Init(i2c.sdaPort, &GPIO_InitStructure);//初始化
}

/*
*********************************************************************************************************
*                             i2cdrv_SetSDAOut             
*
* Description: 设置i2c通信SDA引脚为输出模式
*             
* Arguments  : i2c控制结构体
*
* Reutrn     : None.
*
* Note(s)    : 本文件私有函数
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
	GPIO_Init(i2c.sdaPort, &GPIO_InitStructure);//初始化
}



/*
*********************************************************************************************************
*                           i2cdrv_SetSCLHigh               
*
* Description: 设置SCL脚为高电平
*             
* Arguments  : i2c控制结构体
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
* Description: 设置SCL脚为低电平
*             
* Arguments  : i2c控制结构体
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
* Description: 读取SDA信号线
*             
* Arguments  : 1.i2c: i2c通信控制结构体
*
* Reutrn     : SDA信号线状态
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
* Description: i2c通信启动信号
*             
* Arguments  : 1.i2c: i2c通信控制结构体
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
* Description: 主器件发送停止i2c通信指令
*             
* Arguments  : 1.i2c: i2c控制结构体
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
* Description: 主器件发送响应命令
*             
* Arguments  : 1.i2c: i2c控制结构体
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
* Description: 主器件发送无响应命令
*             
* Arguments  : 1.i2c: i2c控制结构体
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
* Description: 等待器件响应命令
*             
* Arguments  : 1.i2c: i2c控制结构体
*
* Reutrn     : 返回1无响应,0有响应
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
* Description: 通过i2c发送一个字节的数据
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.byte: 要发送的数据,8位类型
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
* Description: i2c读取一个字节的数据
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.ack: 是否需要发送ack, 0,不需要,1需要
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
* Description: 往i2c设备的地址写一个字节的数据
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.byte: 要写入的数据
*
* Reutrn     : 写入成功或者失败
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
* Description: 往i2c设备的地址写多个字节的数据
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.len: 要写入的数据长度
*							 5.buff: 要写入的数据缓存区地址
*
* Reutrn     : 写入成功或者失败
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
* Description: 从i2c设备的某一个地址读取一个字节的数据
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.byte: 数据存储地址
*
* Reutrn     : 读取成功或者失败
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
* Description: 从i2c设备的某一个地址中读取多个字节的数据
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.len: 要读取的数据长度
*							 5.buff: 数据存储地址
*
* Reutrn     : 读取成功或者失败
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
* Description: 写i2c设备的某一个寄存器的某一位
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.bitx: 要写的寄存器位
*							 5.data: 写0或者写1
*
* Reutrn     : 写入后的寄存器值
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t i2c_WriteBit(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitx, uint8_t data)
{
	uint8_t byte = 0;
	
	/*  需要先从寄存器中读取原来的值,防止影响其他位  */
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);
	
	byte = (data == 0) ? (byte & ~(1 << bitx)) : (byte | (1 << bitx));
	return i2c_WriteByte(i2c, devAddr, memAddr, byte);
}

/*
*********************************************************************************************************
*                      i2c_WriteBits                    
*
* Description: 写i2c设备的某一个寄存器的多个位
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.bitStart: 要写的寄存器开始位
*							 5.bitLen: 要写的位长度
*							 6.data: 要写入的数据
*
* Reutrn     : 写入后的寄存器值
*
* Note(s)    : 该函数只能写连续的寄存器位,不能写非连续的位
*********************************************************************************************************
*/
uint8_t i2c_WriteBits(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitStart, uint8_t bitLen, uint8_t data)
{
	uint8_t byte = 0;
	
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);
	if(byte != 0xff)
	{
		uint8_t mask = ((1 << bitLen) - 1) << (bitStart - bitLen + 1);
		data <<= (bitStart - bitLen + 1);	/*  将数据插入到正解的位置  */
		data &= mask;		/* 	清零其他位  */
		byte &= ~mask;	/*  清除原来的位  */
		byte |= data;		/*  合并数据  */
		i2c_WriteByte(i2c, devAddr, memAddr, byte);
	}
	i2c_ReadByte(i2c, devAddr, memAddr, &byte);	/*  读取设置后的寄存器值  */
	return byte;
}


/*
*********************************************************************************************************
*                       i2c_ReadBit                   
*
* Description: 读取i2c设备的某一个寄存器的某一位
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.bitx: 要读取的位
*						   5.data: 用于存储结果的缓存区地址
*
* Reutrn     : 读取成功或者失败
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
* Description: 读i2c设备的寄存器的多个位
*             
* Arguments  : 1.i2c: i2c控制结构体
*							 2.devAddr: 设备地址
*							 3.memAddr: 内存地址
*							 4.bitStart: 要读取的开始位
*							 5.bitLen: 要读取的位长度
*							 6.data: 用于存储结果的缓存区地址
*
* Reutrn     : 读取成功或者失败
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



/**
  *******************************************************************************************************
  * File Name: bsp_ee.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-4
  * Brief: 本文件提供了对主控上EEPROM的基本操作函数
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mode: 建立文件
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
#include "bsp.h"

/*  EEPROM IIC操作结构体  */
IIC_TypeDef ee_iic;


/********************************************  END OF FILE  *******************************************/

/*
*********************************************************************************************************
*                            EE_SDA_IN              
*
* Description: EEPROM设置SDA为输入函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void EE_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = EE_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
  GPIO_Init(EE_SDA_PORT, &GPIO_InitStructure);//初始化
}

/*
*********************************************************************************************************
*                              EE_SDA_OUT            
*
* Description: EEPROM设置SDA引脚为输出模式
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void EE_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = EE_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
  GPIO_Init(EE_SDA_PORT, &GPIO_InitStructure);//初始化
}

/*
*********************************************************************************************************
*                          EE_SDA_LOW                
*
* Description: EEPROM设置SDA引脚为低电平
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void EE_SDA_LOW(void)
{
	EE_SDA_PORT->ODR &= ~EE_SDA_PIN;
}

/*
*********************************************************************************************************
*                             EE_SDA_HIGH             
*
* Description: EEPROM设置SDA引脚为高电平 
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void EE_SDA_HIGH(void)
{
	EE_SDA_PORT->ODR |= EE_SDA_PIN;
}


/*
*********************************************************************************************************
*                         EE_READ_SDA                 
*
* Description: EEPROM读SDA引脚电平
*             
* Arguments  : None.
*
* Reutrn     : 1> SDA引脚电平,0或者1
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t EE_READ_SDA(void)
{
	return ((EE_SDA_PORT->IDR & EE_SDA_PIN) == 0)? 0 : 1;
}

/*
*********************************************************************************************************
*                        EE_SCL_HIGH                  
*
* Description: EEPROM设置SCL引脚为高电平
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void EE_SCL_HIGH(void)
{
	EE_SCL_PORT->ODR |= EE_SCL_PIN;
}

/*
*********************************************************************************************************
*                    EE_SCL_LOW                      
*
* Description: EEPROM设置SCL引脚为低电平
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void EE_SCL_LOW(void)
{
	EE_SCL_PORT->ODR &= ~EE_SCL_PIN;
}

/*
*********************************************************************************************************
*                          bsp_ee_Config                
*
* Description: EEPROM初始化函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_ee_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = EE_SDA_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(EE_SDA_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = EE_SCL_PIN;
	GPIO_Init(EE_SCL_PORT, &GPIO_InitStruct);
	
	ee_iic.read_sda = EE_READ_SDA;
	ee_iic.set_scl_high = EE_SCL_HIGH;
	ee_iic.set_scl_low = EE_SCL_LOW;
	ee_iic.set_sda_high = EE_SDA_HIGH;
	ee_iic.set_sda_in = EE_SDA_IN;
	ee_iic.set_sda_low = EE_SDA_LOW;
	ee_iic.set_sda_out = EE_SDA_OUT;
}

/*
*********************************************************************************************************
*                          bsp_ee_Check                
*
* Description: EEPROM检查函数,验证EEPROM功能是否正常
*             
* Arguments  : None.
*
* Reutrn     : 1> 0: 正常
*              2> 1: 异常
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_ee_Check(void)
{
	u8 temp;
	temp = bsp_ee_ReadShortByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		bsp_ee_WriteShortByte(255,0X55);
		temp = bsp_ee_ReadShortByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;		
}

/*
*********************************************************************************************************
*                          bsp_ee_ReadShortByte               
*
* Description: EEPROM读取一个短字节,uint8_t型
*             
* Arguments  : 1> addr: 数据地址
*
* Reutrn     : 1> 读取到的数据
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_ee_ReadShortByte(uint16_t addr)
{
	uint8_t temp=0;		  	    																 
	bsp_i2c_Start(ee_iic);  
	if(EE_TYPE>AT24C16)
	{
		bsp_i2c_SendByte(ee_iic, 0XA0);	   //发送写命令
		bsp_i2c_Ack(ee_iic);
		bsp_i2c_SendByte(ee_iic, (addr>>8));//发送高地址
		bsp_i2c_Ack(ee_iic);		 
	}else bsp_i2c_SendByte(ee_iic, (0XA0+((addr/256)<<1)));   //发送器件地址0XA0,写数据 	 

	bsp_i2c_Ack(ee_iic); 
    bsp_i2c_SendByte(ee_iic, (addr%256));   //发送低地址
	bsp_i2c_Ack(ee_iic);	    
	bsp_i2c_Start(ee_iic);  	 	   
	bsp_i2c_SendByte(ee_iic, (0XA1));           //进入接收模式			   
	bsp_i2c_Ack(ee_iic);	 
	temp=bsp_i2c_ReadByte(ee_iic, 0);		   
	bsp_i2c_Stop(ee_iic);//产生一个停止条件	    
	return temp;
}

/*
*********************************************************************************************************
*                           bsp_ee_ReadLongByte               
*
* Description: EEPROM读取一个长字节数据, uint16_t或者uint32_t型
*             
* Arguments  : 1> addr: 数据地址
*              2> bytelen: 要读取的字节长度,2或者4
*
* Reutrn     : 1> 读取到的数据
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint32_t bsp_ee_ReadLongByte(uint16_t addr, uint8_t bytelen)
{
	uint8_t t;
	uint32_t temp=0;
	for(t = 0;t < bytelen;t++)
	{
		temp<<=8;
		temp+=bsp_ee_ReadShortByte(addr + bytelen - t - 1); 	 				   
	}
	return temp;		
}


/*
*********************************************************************************************************
*                           bsp_ee_WriteShortByte               
*
* Description: EEPROM写一个短字节数据,uint8_t
*             
* Arguments  : 1> addr: 要写入的数据地址
*							 2> byte: 要写入的数据
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_ee_WriteShortByte(uint16_t addr, uint8_t byte)
{
	bsp_i2c_Start(ee_iic);  
	if(EE_TYPE>AT24C16)
	{
		bsp_i2c_SendByte(ee_iic, (0XA0));	    //发送写命令
		bsp_i2c_Ack(ee_iic);
		bsp_i2c_SendByte(ee_iic, (addr >> 8));//发送高地址
 	}else
	{
		bsp_i2c_SendByte(ee_iic, (0XA0+((addr/256)<<1)));   //发送器件地址0XA0,写数据 
	}	 
	bsp_i2c_Ack(ee_iic);	   
    bsp_i2c_SendByte(ee_iic, (addr%256));   //发送低地址
	bsp_i2c_Ack(ee_iic); 	 										  		   
	bsp_i2c_SendByte(ee_iic, (byte));     //发送字节							   
	bsp_i2c_Ack(ee_iic);  		    	   
	bsp_i2c_Stop(ee_iic);//产生一个停止条件 
	bsp_tim_DelayMs(10);	 
}

/*
*********************************************************************************************************
*                              bsp_ee_writeLongByte            
*
* Description: EEPROM写入一个长字节数据,uint16_t或者uint32_t
*             
* Arguments  : 1> addr: 写入地址
*              2> byte: 数据
*              3> bytelen: 数据字节长度, 2或者4
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_ee_writeLongByte(uint16_t addr, uint32_t byte, uint8_t bytelen)
{
	u8 t;
	for(t=0;t<bytelen;t++)
	{
		bsp_ee_WriteShortByte(addr+t,(byte>>(8*t))&0xff);
	}		
}


/*
*********************************************************************************************************
*                           bsp_ee_ReadBytes               
*
* Description: EEPROM读取多个字节
*             
* Arguments  : 1> addr: 数据地址
*              2> bytes: 数据缓存区
*              3> len: 要读取的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_ee_ReadBytes(uint16_t addr, uint8_t *bytes, uint16_t len)
{
	while(len)
	{
		*bytes++=bsp_ee_ReadShortByte(addr++);	
		len--;
	}
}


/*
*********************************************************************************************************
*                        bsp_ee_WriteBytes                  
*
* Description: EEPROM写入多个字节
*             
* Arguments  : 1> addr: 数据地址
*              2> bytes: 数据缓存区
*              3> len: 数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_ee_WriteBytes(uint16_t addr, uint8_t *bytes, uint16_t len)
{
	while(len--)
	{
		bsp_ee_WriteShortByte(addr,*bytes);
		addr++;
		bytes++;
	}
}


/********************************************  END OF FILE  *******************************************/

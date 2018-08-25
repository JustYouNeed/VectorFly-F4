/**
  *******************************************************************************************************
  * File Name: i2cdrv.h
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-3-4
  * Brief: 本文件提供了IIC通信的软件模拟驱动
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: 建立本文件
	*		
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: 增加延时函数宏,使驱动更容易移植
	*
	*		3.Author: Vector
	*			Date: 2018-8-23
	*			Mod: 修改IIC控制逻辑,修改原来IIC控制结构体,更容易移植
  *
  *******************************************************************************************************
  */	

# ifndef __IIC_DRV_H
# define __IIC_DRV_H

/*  i2c控制结构体  */
typedef struct
{
	GPIO_TypeDef*	sdaPort;
	uint32_t 		 	sdaPin;
	GPIO_TypeDef*	sclPort;
	uint32_t			sclPin;
}i2cDevice;

/*
  *******************************************************************************************************
  *                              FUNCTION DECLARE
  *******************************************************************************************************
*/
uint8_t i2c_WriteByte(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t byte);
uint8_t i2c_WriteBuff(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t len, uint8_t *buff);

uint8_t i2c_ReadByte(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t *byte);
uint8_t i2c_ReadBuff(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t len, uint8_t *buff);

uint8_t i2c_WriteBit(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitx, uint8_t data);
uint8_t i2c_WriteBits(i2cDevice i2c, uint8_t devAddrm, uint8_t memAddr, uint8_t bitStart, uint8_t bitLen, uint8_t data);

uint8_t i2c_ReadBit(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitx, uint8_t *data);
uint8_t i2c_ReadBits(i2cDevice i2c, uint8_t devAddr, uint8_t memAddr, uint8_t bitStart, uint8_t bitLen, uint8_t *data);

# endif

/********************************************  END OF FILE  *******************************************/






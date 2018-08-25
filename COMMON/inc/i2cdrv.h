/**
  *******************************************************************************************************
  * File Name: i2cdrv.h
  * Author: Vector
  * Version: V2.1.0
  * Date: 2018-3-4
  * Brief: ���ļ��ṩ��IICͨ�ŵ����ģ������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mod: �������ļ�
	*		
	*		2.Author: Vector
	*			Date: 2018-5-4
	*			Mod: ������ʱ������,ʹ������������ֲ
	*
	*		3.Author: Vector
	*			Date: 2018-8-23
	*			Mod: �޸�IIC�����߼�,�޸�ԭ��IIC���ƽṹ��,��������ֲ
  *
  *******************************************************************************************************
  */	

# ifndef __IIC_DRV_H
# define __IIC_DRV_H

/*  i2c���ƽṹ��  */
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






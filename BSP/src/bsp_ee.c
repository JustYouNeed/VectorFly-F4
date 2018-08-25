/**
  *******************************************************************************************************
  * File Name: bsp_ee.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-4
  * Brief: ���ļ��ṩ�˶�������EEPROM�Ļ�����������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-4
	*			Mode: �����ļ�
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
#include "bsp.h"

/*  EEPROM IIC�����ṹ��  */
IIC_TypeDef ee_iic;


/********************************************  END OF FILE  *******************************************/

/*
*********************************************************************************************************
*                            EE_SDA_IN              
*
* Description: EEPROM����SDAΪ���뺯��
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
  GPIO_Init(EE_SDA_PORT, &GPIO_InitStructure);//��ʼ��
}

/*
*********************************************************************************************************
*                              EE_SDA_OUT            
*
* Description: EEPROM����SDA����Ϊ���ģʽ
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
  GPIO_Init(EE_SDA_PORT, &GPIO_InitStructure);//��ʼ��
}

/*
*********************************************************************************************************
*                          EE_SDA_LOW                
*
* Description: EEPROM����SDA����Ϊ�͵�ƽ
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
* Description: EEPROM����SDA����Ϊ�ߵ�ƽ 
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
* Description: EEPROM��SDA���ŵ�ƽ
*             
* Arguments  : None.
*
* Reutrn     : 1> SDA���ŵ�ƽ,0����1
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
* Description: EEPROM����SCL����Ϊ�ߵ�ƽ
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
* Description: EEPROM����SCL����Ϊ�͵�ƽ
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
* Description: EEPROM��ʼ������
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
* Description: EEPROM��麯��,��֤EEPROM�����Ƿ�����
*             
* Arguments  : None.
*
* Reutrn     : 1> 0: ����
*              2> 1: �쳣
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_ee_Check(void)
{
	u8 temp;
	temp = bsp_ee_ReadShortByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
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
* Description: EEPROM��ȡһ�����ֽ�,uint8_t��
*             
* Arguments  : 1> addr: ���ݵ�ַ
*
* Reutrn     : 1> ��ȡ��������
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
		bsp_i2c_SendByte(ee_iic, 0XA0);	   //����д����
		bsp_i2c_Ack(ee_iic);
		bsp_i2c_SendByte(ee_iic, (addr>>8));//���͸ߵ�ַ
		bsp_i2c_Ack(ee_iic);		 
	}else bsp_i2c_SendByte(ee_iic, (0XA0+((addr/256)<<1)));   //����������ַ0XA0,д���� 	 

	bsp_i2c_Ack(ee_iic); 
    bsp_i2c_SendByte(ee_iic, (addr%256));   //���͵͵�ַ
	bsp_i2c_Ack(ee_iic);	    
	bsp_i2c_Start(ee_iic);  	 	   
	bsp_i2c_SendByte(ee_iic, (0XA1));           //�������ģʽ			   
	bsp_i2c_Ack(ee_iic);	 
	temp=bsp_i2c_ReadByte(ee_iic, 0);		   
	bsp_i2c_Stop(ee_iic);//����һ��ֹͣ����	    
	return temp;
}

/*
*********************************************************************************************************
*                           bsp_ee_ReadLongByte               
*
* Description: EEPROM��ȡһ�����ֽ�����, uint16_t����uint32_t��
*             
* Arguments  : 1> addr: ���ݵ�ַ
*              2> bytelen: Ҫ��ȡ���ֽڳ���,2����4
*
* Reutrn     : 1> ��ȡ��������
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
* Description: EEPROMдһ�����ֽ�����,uint8_t
*             
* Arguments  : 1> addr: Ҫд������ݵ�ַ
*							 2> byte: Ҫд�������
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
		bsp_i2c_SendByte(ee_iic, (0XA0));	    //����д����
		bsp_i2c_Ack(ee_iic);
		bsp_i2c_SendByte(ee_iic, (addr >> 8));//���͸ߵ�ַ
 	}else
	{
		bsp_i2c_SendByte(ee_iic, (0XA0+((addr/256)<<1)));   //����������ַ0XA0,д���� 
	}	 
	bsp_i2c_Ack(ee_iic);	   
    bsp_i2c_SendByte(ee_iic, (addr%256));   //���͵͵�ַ
	bsp_i2c_Ack(ee_iic); 	 										  		   
	bsp_i2c_SendByte(ee_iic, (byte));     //�����ֽ�							   
	bsp_i2c_Ack(ee_iic);  		    	   
	bsp_i2c_Stop(ee_iic);//����һ��ֹͣ���� 
	bsp_tim_DelayMs(10);	 
}

/*
*********************************************************************************************************
*                              bsp_ee_writeLongByte            
*
* Description: EEPROMд��һ�����ֽ�����,uint16_t����uint32_t
*             
* Arguments  : 1> addr: д���ַ
*              2> byte: ����
*              3> bytelen: �����ֽڳ���, 2����4
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
* Description: EEPROM��ȡ����ֽ�
*             
* Arguments  : 1> addr: ���ݵ�ַ
*              2> bytes: ���ݻ�����
*              3> len: Ҫ��ȡ�����ݳ���
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
* Description: EEPROMд�����ֽ�
*             
* Arguments  : 1> addr: ���ݵ�ַ
*              2> bytes: ���ݻ�����
*              3> len: ���ݳ���
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

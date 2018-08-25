# include "app_24cxx.h"

I2C_Str AT24CXX;
void SDA_IN(void)
{
	{GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
}

void SDA_OUT(void)
{
	{GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ
}

uint8_t READ_SDA(void)
{
	return PBin(9);
}

void SDA_HIGH(void)
{
	PBout(9) = 1;
}
void SDA_LOW(void)
{
	PBout(9) = 0;
}

void SCL_HIGH(void)
{
	PBout(8) = 1;
}
void SCL_LOW(void)
{
	PBout(8) = 0;
}
void AT_GPIOConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	AT24CXX.read_sda = READ_SDA;
	AT24CXX.set_scl_high = SCL_HIGH;
	AT24CXX.set_scl_low = SCL_LOW;
	AT24CXX.set_sda_high = SDA_HIGH;
	AT24CXX.set_sda_in = SDA_IN;
	AT24CXX.set_sda_low = SDA_LOW;
	AT24CXX.set_sda_out = SDA_OUT;
}



//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
uint8_t AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    bsp_i2cStart(AT24CXX);  
	if(EE_TYPE>AT24C16)
	{
		bsp_i2cSendByte(AT24CXX, 0XA0);	   //����д����
		bsp_i2cWaitAck(AT24CXX);
		bsp_i2cSendByte(AT24CXX, ReadAddr>>8);//���͸ߵ�ַ	    
	}else bsp_i2cSendByte(AT24CXX, 0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	   
	bsp_i2cWaitAck(AT24CXX); 
    bsp_i2cSendByte(AT24CXX, ReadAddr%256);   //���͵͵�ַ
	bsp_i2cWaitAck(AT24CXX);	    
	bsp_i2cStart(AT24CXX);  	 	   
	bsp_i2cSendByte(AT24CXX,0XA1);           //�������ģʽ			   
	bsp_i2cWaitAck(AT24CXX);	 
    temp=bsp_i2cReadByte(AT24CXX,0);		   
    bsp_i2cStop(AT24CXX);//����һ��ֹͣ����	    
	return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    bsp_i2cStart(AT24CXX);  
	if(EE_TYPE>AT24C16)
	{
		bsp_i2cSendByte(AT24CXX, 0XA0);	    //����д����
		bsp_i2cWaitAck(AT24CXX);
		bsp_i2cSendByte(AT24CXX, WriteAddr>>8);//���͸ߵ�ַ	  
	}else bsp_i2cSendByte(AT24CXX, 0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 	 
	bsp_i2cWaitAck(AT24CXX);	   
    bsp_i2cSendByte(AT24CXX, WriteAddr%256);   //���͵͵�ַ
	bsp_i2cWaitAck(AT24CXX); 	 										  		   
	bsp_i2cSendByte(AT24CXX, DataToWrite);     //�����ֽ�							   
	bsp_i2cWaitAck(AT24CXX);  		    	   
    bsp_i2cStop(AT24CXX);//����һ��ֹͣ���� 
	bsp_DelayMs(10);	 
}
//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
uint32_t AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}


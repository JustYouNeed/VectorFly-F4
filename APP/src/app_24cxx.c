# include "app_24cxx.h"

I2C_Str AT24CXX;
void SDA_IN(void)
{
	{GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
}

void SDA_OUT(void)
{
	{GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
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

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	AT24CXX.read_sda = READ_SDA;
	AT24CXX.set_scl_high = SCL_HIGH;
	AT24CXX.set_scl_low = SCL_LOW;
	AT24CXX.set_sda_high = SDA_HIGH;
	AT24CXX.set_sda_in = SDA_IN;
	AT24CXX.set_sda_low = SDA_LOW;
	AT24CXX.set_sda_out = SDA_OUT;
}



//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    bsp_i2cStart(AT24CXX);  
	if(EE_TYPE>AT24C16)
	{
		bsp_i2cSendByte(AT24CXX, 0XA0);	   //发送写命令
		bsp_i2cWaitAck(AT24CXX);
		bsp_i2cSendByte(AT24CXX, ReadAddr>>8);//发送高地址	    
	}else bsp_i2cSendByte(AT24CXX, 0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	   
	bsp_i2cWaitAck(AT24CXX); 
    bsp_i2cSendByte(AT24CXX, ReadAddr%256);   //发送低地址
	bsp_i2cWaitAck(AT24CXX);	    
	bsp_i2cStart(AT24CXX);  	 	   
	bsp_i2cSendByte(AT24CXX,0XA1);           //进入接收模式			   
	bsp_i2cWaitAck(AT24CXX);	 
    temp=bsp_i2cReadByte(AT24CXX,0);		   
    bsp_i2cStop(AT24CXX);//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    bsp_i2cStart(AT24CXX);  
	if(EE_TYPE>AT24C16)
	{
		bsp_i2cSendByte(AT24CXX, 0XA0);	    //发送写命令
		bsp_i2cWaitAck(AT24CXX);
		bsp_i2cSendByte(AT24CXX, WriteAddr>>8);//发送高地址	  
	}else bsp_i2cSendByte(AT24CXX, 0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 	 
	bsp_i2cWaitAck(AT24CXX);	   
    bsp_i2cSendByte(AT24CXX, WriteAddr%256);   //发送低地址
	bsp_i2cWaitAck(AT24CXX); 	 										  		   
	bsp_i2cSendByte(AT24CXX, DataToWrite);     //发送字节							   
	bsp_i2cWaitAck(AT24CXX);  		    	   
    bsp_i2cStop(AT24CXX);//产生一个停止条件 
	bsp_DelayMs(10);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
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
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}


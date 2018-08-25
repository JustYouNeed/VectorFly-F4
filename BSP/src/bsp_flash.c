/**
  *******************************************************************************************************
  * File Name: bsp_flash.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-19
  * Brief: SPI-Flash驱动模块,使用芯片W25Q128
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-19
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*  芯片型号ID  */
# define W25Q80 	0XEF13 	
# define W25Q16 	0XEF14
# define W25Q32 	0XEF15
# define W25Q64 	0XEF16
# define W25Q128	0XEF17


/*  寄存器操作指令  */
# define W25X_WriteEnable				0x06 
# define W25X_WriteDisable			0x04 
# define W25X_ReadStatusReg			0x05 
# define W25X_WriteStatusReg		0x01 
# define W25X_ReadData					0x03 
# define W25X_FastReadData			0x0B 
# define W25X_FastReadDual			0x3B 
# define W25X_PageProgram				0x02 
# define W25X_BlockErase				0xD8 
# define W25X_SectorErase				0x20 
# define W25X_ChipErase					0xC7 
# define W25X_PowerDown					0xB9 
# define W25X_ReleasePowerDown	0xAB 
# define W25X_DeviceID					0xAB 
# define W25X_ManufactDeviceID	0x90 
# define W25X_JedecDeviceID			0x9F 
# define W25X_NOP								0x00
# define W25X_DUMMY							0xff

/*  FLASH大小16M  */
# define FLASH_SIZE			1024 * 1024 * 16		

/*  定义Flash片选引脚  */
# define FLASH_CS_PORT		GPIOA
# define FLASH_CS_PIN			GPIO_Pin_15

/*  FLASH的SPI通道  */
# define FLASH_SPI				SPI1

/*  缓存区大小  */
# define FLASH_BUFF_LEN		4096		

/*  是否开启DMA传输  */
# define FLASH_USE_DMA		0u

/*  是否开启DMA中断  */
# define FLASH_DMA_IRQN		0u

/*  SPI1的外设基地址  */
# define SPI1_DR_ADDR    (uint32_t)0x4001300C

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
uint16_t FLASH_TYPE = W25Q128;					/*  FLASH型号,用于判断是否初始化成功  */
uint8_t *FLASH_TX_BUFF;	/*  发送缓存区  */
uint8_t *FLASH_RX_BUFF;	/*  接收缓存区  */

/*
*********************************************************************************************************
*                          bsp_flash_Select                
*
* Description: 选中芯片
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_flash_Select(void)
{
	PIN_OUT_LOW(FLASH_CS_PORT, FLASH_CS_PIN);
}

/*
*********************************************************************************************************
*                         bsp_flash_Unselect                 
*
* Description: 取消选中芯片
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_flash_Unselect(void)
{
	PIN_OUT_HIGH(FLASH_CS_PORT, FLASH_CS_PIN);
}

/*
*********************************************************************************************************
*                      bsp_flash_GPIOConfig                    
*
* Description: 初始化Flash芯片片选引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_flash_GPIOConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	bsp_rcc_GPIOClcokCmd(FLASH_CS_PORT);
	
//	GPIO_InitStructure.GPIO_Pin = FLASH_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(FLASH_CS_PORT, &GPIO_InitStructure);
}

# if FLASH_USE_DMA > 0u
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
static void spi_DMAConfig(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*  使能DMA时钟  */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          /*  DMA通道3  */
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                /*  外设基地址  */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)FLASH_RX_BUFF;        /*  接收缓存区地址  */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 /*  接收是外设到内存  */
	DMA_InitStructure.DMA_BufferSize = FLASH_BUFF_LEN;                      /*  传输数据长度  */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        /*  外设地址不自增  */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 /*  内存地址自增  */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         /*  数据大小为8位  */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /*  外设数据为8位  */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           /*  不采用循环模式  */
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   /*  优先级中等  */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  /*  不采取FIFO  */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           /*  无效  */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             /*  单次传输  */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     /*  单次传输  */
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)FLASH_TX_BUFF;        /*  发送缓存区地址  */
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 /*  发送是内存到外设  */
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);
	
# if FLASH_DMA_IRQN > 0u
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;		/*  发送中断  */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;		/*  接收中断  */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);
# endif
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
}

# if FLASH_DMA_IRQN > 0u
/*
*********************************************************************************************************
*                             DMA2_Stream2_IRQHandler             
*
* Description: DMA2数据流2的中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 在这里只处理SPI1DMA接收完成中断
*********************************************************************************************************
*/
void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) == SET)
	{
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	}
}

/*
*********************************************************************************************************
*                             DMA2_Stream3_IRQHandler             
*
* Description: DMA2数据流3的中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 在这里只处理SPI1DMA发送完成中断
*********************************************************************************************************
*/
void DMA2_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3) == SET)
	{
		DMA_Cmd(DMA2_Stream3, DISABLE);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	}
}
# endif
# endif

/*
*********************************************************************************************************
*                          bsp_flash_SPIConfig                
*
* Description: 初始化Flash芯片所使用的SPI接口
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_flash_SPIConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
	bsp_rcc_SPIClockCmd(FLASH_SPI);
	bsp_rcc_GPIOClcokCmd(GPIOB);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//PB3~5复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //PB3复用为 SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1); //PB4复用为 SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1); //PB5复用为 SPI1

 	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(FLASH_SPI, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

# if FLASH_USE_DMA > 0u
	SPI_I2S_DMACmd(FLASH_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(FLASH_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
	spi_DMAConfig();
# endif
 
	SPI_Cmd(FLASH_SPI, ENABLE); /*  使能SPI外设  */
	SPI_I2S_ClearITPendingBit(FLASH_SPI, SPI_I2S_IT_RXNE);		/*  清除接收标志位  */
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
static void flash_SPI_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	FLASH_SPI->CR1&=0XFFC7;
	FLASH_SPI->CR1|=SPI_BaudRatePrescaler;	
	SPI_Cmd(FLASH_SPI,ENABLE);
}

# if FLASH_USE_DMA > 0u
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
static void flash_SPI_DMAStart(void)
{
	/*  开启DMA  */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream3, ENABLE);
	
# if FLASH_DMA_IRQN < 1u
	/*  等待传输完成  */
	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/*  关闭DMA通道  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);	
	
	/*  清除标志  */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
# endif
}
/*
*********************************************************************************************************
*                            flash_SPI_ReadWrite              
*
* Description: 通过SPI同时读写Flash,在写入的同时返回数据
*             
* Arguments  : 1> rx_buff: 接收缓存区
*							 2> tx_buff: 发送缓存区
*							 3> len: 要发送和接收的数据长度
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
static void flash_SPI_ReadWrite(uint8_t *rx_buff, uint8_t *tx_buff, uint16_t len)
{
	/*  在设置传输字节数之前需要先关闭DMA  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)len);	/*  设置要传输的字节数  */
	DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)len);
	
	/*  发送和接收的数据地址都要自增  */
	DMA2_Stream2->CR |= (1 << 10);
	DMA2_Stream3->CR |= (1 << 10);
	
	/*  设置发送和接收的内存地址  */
	DMA2_Stream2->M0AR = (uint32_t)rx_buff;
	DMA2_Stream3->M0AR = (uint32_t)tx_buff;
	
	(void)FLASH_SPI->DR;	/*  清空SPI数据寄存器  */
	
	/*  等待发送区为空  */
	while(SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
		
	flash_SPI_DMAStart();		/*  开启DMA传输  */
}

/*
*********************************************************************************************************
*                          flash_SPI_Write                
*
* Description: 通过SPI读取Flash
*             
* Arguments  : 1> tx_buff: 发送数据缓存区
*							 2> len: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void flash_SPI_Write(uint8_t *tx_buff, uint16_t len)
{
	uint8_t temp[1];
	
	/*  在设置传输字节数之前需要先关闭DMA  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	
	DMA_SetCurrDataCounter(DMA2_Stream2, 1);	/*  设置要传输的字节数  */
	DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)len);
	
	/*  发送的需要自增,接收的不需要,因为这里不接收  */
	DMA2_Stream2->CR &= ~(1 << 10);
	
	/*  设置发送和接收的内存地址  */
	DMA2_Stream2->M0AR = (uint32_t)temp;
	DMA2_Stream3->M0AR = (uint32_t)tx_buff;
	
	(void)FLASH_SPI->DR;	/*  清空SPI数据寄存器  */
	
	/*  等待发送区为空  */
	while(SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
	
	flash_SPI_DMAStart();		/*  开启DMA传输  */
}

/*
*********************************************************************************************************
*                             flash_SPI_Read             
*
* Description: 通过SPI读取Flash
*             
* Arguments  : 1> rx_buff: 读取缓存区
*							 2> len: 要读取的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void flash_SPI_Read(uint8_t *rx_buff, uint16_t len)
{
	uint8_t temp[1];
	
	/*  在设置传输字节数之前需要先关闭DMA  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)len);	/*  设置要传输的字节数  */
	DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)len);
	
	/*  接收的需要自增,发送的不需要  */
	DMA2_Stream3->CR &= ~(1 << 10);
	
	/*  设置发送和接收的内存地址  */
	DMA2_Stream2->M0AR = (uint32_t)rx_buff;
	DMA2_Stream3->M0AR = (uint32_t)temp;
	
	(void)FLASH_SPI->DR;	/*  清空SPI数据寄存器  */
	
	/*  等待发送区为空  */
	while(SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
	
	flash_SPI_DMAStart();		/*  开启DMA传输  */
}
# else
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
static uint8_t flash_SPI_ReadWriteByte(uint8_t byte)
{	
	while (SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);	
	SPI_I2S_SendData(FLASH_SPI, byte); //通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);  
	return SPI_I2S_ReceiveData(FLASH_SPI); //返回通过SPIx最近接收的数据
}
/*
*********************************************************************************************************
*                            flash_SPI_ReadWrite              
*
* Description: 通过SPI同时读写Flash,在写入的同时返回数据
*             
* Arguments  : 1> rx_buff: 接收缓存区
*							 2> tx_buff: 发送缓存区
*							 3> len: 要发送和接收的数据长度
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
static void flash_SPI_ReadWrite(uint8_t *rx_buff, uint8_t *tx_buff, uint16_t len)
{
	uint16_t i = 0;
	
	for(i = 0; i < len; i ++)
	{
		rx_buff[i] = flash_SPI_ReadWriteByte(tx_buff[i]);
	}
}
/*
*********************************************************************************************************
*                          flash_SPI_Write                
*
* Description: 通过SPI读取Flash
*             
* Arguments  : 1> tx_buff: 发送数据缓存区
*							 2> len: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void flash_SPI_Write(uint8_t *tx_buff, uint16_t len)
{
	uint16_t i = 0;
	
	for(i = 0; i < len; i ++)
	{
		flash_SPI_ReadWriteByte(tx_buff[i]);
	}
}

/*
*********************************************************************************************************
*                             flash_SPI_Read             
*
* Description: 通过SPI读取Flash
*             
* Arguments  : 1> rx_buff: 读取缓存区
*							 2> len: 要读取的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void flash_SPI_Read(uint8_t *rx_buff, uint16_t len)
{
	uint16_t i = 0;
	
	for(i = 0; i < len; i ++)
	{
		rx_buff[i] = flash_SPI_ReadWriteByte(0xff);
	}
}
# endif

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
uint8_t 			bsp_flash_Config(void)
{
	bsp_flash_GPIOConfig();
	bsp_flash_SPIConfig();
	flash_SPI_SetSpeed(SPI_BaudRatePrescaler_4);
	
	bsp_flash_Unselect();
	
	FLASH_TX_BUFF = (uint8_t *)bsp_mem_Alloc(SRAMIN, FLASH_BUFF_LEN * sizeof(uint8_t));
	FLASH_RX_BUFF = (uint8_t *)bsp_mem_Alloc(SRAMIN, FLASH_BUFF_LEN * sizeof(uint8_t));
	
	if(bsp_flash_ReadID() == FLASH_TYPE) return 0;
	
	return 1;
}

/*
*********************************************************************************************************
*                       bsp_flash_ReadID                   
*
* Description: 读取FLASH的ID
*             
* Arguments  : None.
*
* Reutrn     : 读取到的ID
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint16_t 	bsp_flash_ReadID(void)
{
	uint16_t id = 0x00;
	uint8_t cnt = 0;
	
	FLASH_TX_BUFF[cnt ++] = W25X_ManufactDeviceID;
	FLASH_TX_BUFF[cnt ++] = W25X_NOP;
	FLASH_TX_BUFF[cnt ++] = W25X_NOP;
	FLASH_TX_BUFF[cnt ++] = W25X_NOP;
	FLASH_TX_BUFF[cnt ++] = W25X_DUMMY;
	FLASH_TX_BUFF[cnt ++] = W25X_DUMMY;
	
	bsp_flash_Select();
	flash_SPI_ReadWrite(FLASH_RX_BUFF, FLASH_TX_BUFF, cnt);	
	bsp_flash_Unselect();
	
	id = (FLASH_RX_BUFF[4] << 8 | FLASH_RX_BUFF[5]);
	
	return id;
}

/*
*********************************************************************************************************
*                          bsp_flash_GetFlashSize                
*
* Description: 读取Flash的内存大小
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint32_t 	bsp_flash_GetFlashSize(void)
{
	return FLASH_SIZE;
}

/*
*********************************************************************************************************
*                            bsp_flash_ReadSR              
*
* Description: 读取Flash的状态寄存器
*             
* Arguments  : None.
*
* Reutrn     : 读取的到Flash状态寄存器的值
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t 	bsp_flash_ReadSR(void)
{
	uint8_t status = 0;
	uint8_t cnt = 0;
	
	FLASH_TX_BUFF[cnt ++] = W25X_ReadStatusReg;
	FLASH_TX_BUFF[cnt ++] = W25X_DUMMY;
	
	bsp_flash_Select();
	flash_SPI_ReadWrite(FLASH_RX_BUFF, FLASH_TX_BUFF, cnt);	
	bsp_flash_Unselect();
	
	status = FLASH_RX_BUFF[1];	
	return status;
}

/*
*********************************************************************************************************
*                         bsp_flash_WriteSR                 
*
* Description: 写Flash状态寄存器
*             
* Arguments  : 1> sr: 状态寄存器值
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_WriteSR(uint8_t sr)
{
	uint8_t cnt = 0;
	
	FLASH_TX_BUFF[cnt ++] = W25X_WriteStatusReg;
	FLASH_TX_BUFF[cnt ++] = W25X_DUMMY;
	
	bsp_flash_Select();
	flash_SPI_Write(FLASH_TX_BUFF, cnt);	
	bsp_flash_Unselect();
}

/*
*********************************************************************************************************
*                       bsp_flash_WriteCmd                   
*
* Description: Flash写使能函数
*             
* Arguments  : 1> state: ENABLE写使能, DISABLE:写失能
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_WriteCmd(FunctionalState state)
{
	uint8_t cnt = 0;
	
	if(DISABLE == state) 
//		FLASH_TX_BUFF[cnt ++] = W25X_WriteDisable;
		flash_SPI_ReadWriteByte(W25X_WriteDisable);
	else 
//		FLASH_TX_BUFF[cnt ++] = W25X_WriteEnable;
		flash_SPI_ReadWriteByte(W25X_WriteEnable);
	
//	bsp_flash_Select();
//	flash_SPI_ReadWrite(FLASH_RX_BUFF, FLASH_TX_BUFF, cnt);
//	bsp_flash_Unselect();
}


/*
*********************************************************************************************************
*                      bsp_flash_WritePage                    
*
* Description: Flash芯片页写函数
*             
* Arguments  : 1> buff: 数据缓存区
*							 2> addr: 要写入的地址
*							 3> len: 数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_WritePage(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint16_t cnt = 0;
	
	FLASH_TX_BUFF[cnt ++] = W25X_PageProgram;
	FLASH_TX_BUFF[cnt ++] = (uint8_t)(addr>>16);
	FLASH_TX_BUFF[cnt ++] = (uint8_t)(addr>>8);
	FLASH_TX_BUFF[cnt ++] = (uint8_t)(addr>>0);
	
	bsp_flash_WriteCmd(ENABLE);		/*  写使能  */
	
	bsp_flash_Select();
	flash_SPI_Write(FLASH_TX_BUFF, cnt);
	flash_SPI_Write(buff, len);
	bsp_flash_Unselect();
	
	bsp_flash_WaitBusy();			/*  等待写入结束  */
}

/*
*********************************************************************************************************
*                         bsp_flash_WriteNoCheck                 
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
void 			bsp_flash_WriteNoCheck(uint8_t *buff, uint32_t addr, uint16_t len)
{
	u16 pageremain;	   
	pageremain=256-addr%256; //单页剩余的字节数		 	    
	if(len<=pageremain)pageremain=len;//不大于256个字节
	while(1)
	{	   
		bsp_flash_WritePage(buff,addr,pageremain);
		if(len==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			buff+=pageremain;
			addr+=pageremain;	

			len-=pageremain;			  //减去已经写入了的字节数
			if(len>256)pageremain=256; //一次可以写入256个字节
			else pageremain=len; 	  //不够256个字节了
		}
	}
}


/*
*********************************************************************************************************
*                        bsp_flash_Read                  
*
* Description: 读取Flash函数
*             
* Arguments  : 1> buff: 数据缓存区
*							 2> addr: 要读取的地址
*							 3> len: 要读取的数据长度
*	
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_Read(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint16_t cnt = 0;
	
	FLASH_TX_BUFF[cnt ++] = W25X_ReadData;
	FLASH_TX_BUFF[cnt ++] = (uint8_t)(addr>>16);
	FLASH_TX_BUFF[cnt ++] = (uint8_t)(addr>>8);
	FLASH_TX_BUFF[cnt ++] = (uint8_t)(addr>>0);
	
	bsp_flash_Select();
	flash_SPI_Write(FLASH_TX_BUFF, cnt);
	flash_SPI_Read(buff, len);	
	bsp_flash_Unselect();
}


/*
*********************************************************************************************************
*                        bsp_flash_Write                  
*
* Description: Flash写数据函数
*             
* Arguments  : 1> buff: 数据缓存区
*							 2> addr: 要写入的地址
*							 3> len: 数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_Write(uint8_t *buff, uint32_t addr, uint16_t len)
{
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	
 	secpos = addr/4096;//扇区地址  
	secoff = addr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
	
 	if(len <= secremain) secremain = len;//不大于4096个字节
	while(1) 
	{	
		bsp_flash_Read(FLASH_TX_BUFF, secpos*4096, 4096);//读出整个扇区的内容
		for(i = 0; i < secremain; i++)//校验数据
		{
			if(FLASH_TX_BUFF[secoff+i] != 0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			bsp_flash_EraseSector(secpos);//擦除这个扇区
			for(i = 0; i < secremain; i++)	   //复制
			{
				FLASH_TX_BUFF[i+secoff] = *(buff + i);	  
			}
			bsp_flash_WriteNoCheck(FLASH_TX_BUFF, secpos*4096,4096);//写入整个扇区  

		}else bsp_flash_WriteNoCheck(buff, addr, secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(len == secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

			buff += secremain;  //指针偏移
			addr += secremain;//写地址偏移	   
			len -= secremain;				//字节数递减
			if(len > 4096)secremain = 4096;	//下一个扇区还是写不完
			else secremain = len;			//下一个扇区可以写完了
		}	 
	}
}


/*
*********************************************************************************************************
*                         bsp_flash_EraseChip                 
*
* Description: 全片擦除Flash
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None
*********************************************************************************************************
*/
void 			bsp_flash_EraseChip(void)
{
	uint8_t cnt = 0;
	
	FLASH_TX_BUFF[cnt ++] = W25X_ChipErase;
	
	bsp_flash_WriteCmd(ENABLE);                  //SET WEL 
	bsp_flash_WaitBusy();   
	bsp_flash_Select();    	//使能器件   
	flash_SPI_Write(FLASH_TX_BUFF, cnt);
	bsp_flash_Unselect();                           //取消片选     	      
	bsp_flash_WaitBusy();   				   //等待芯片擦除结束
}


/*
*********************************************************************************************************
*                     bsp_flash_EraseSector                     
*
* Description: 块擦除函数
*             
* Arguments  : 1> dtsAddr: 要擦除的地址
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_EraseSector(uint32_t dstAddr)
{
	uint8_t cnt = 0;
	
	dstAddr*=4096;
	FLASH_TX_BUFF[cnt ++] = W25X_SectorErase;
	FLASH_TX_BUFF[cnt ++] = (uint8_t)((dstAddr)>>16);
	FLASH_TX_BUFF[cnt ++] = (uint8_t)((dstAddr)>>8);
	FLASH_TX_BUFF[cnt ++] = (uint8_t)dstAddr;
	
	bsp_flash_WriteCmd(ENABLE);                  //SET WEL 	 
	bsp_flash_WaitBusy();   
	bsp_flash_Select();                              //使能器件   
	flash_SPI_Write(FLASH_TX_BUFF, cnt); 
	bsp_flash_Unselect();                            //取消片选     	      
	bsp_flash_WaitBusy();   				   //等待擦除完成
}


/*
*********************************************************************************************************
*                        bsp_flash_WaitBusy                  
*
* Description: 等待Flash操作结束
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_WaitBusy(void)
{
	while((bsp_flash_ReadSR() & 0x01)==0x01);
}


/*
*********************************************************************************************************
*                        bsp_flash_PowerCmd                  
*
* Description: Flash上电函数
*             
* Arguments  : 1> state: ENABLE:上电, DISABLE: 掉电
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void 			bsp_flash_PowerCmd(FunctionalState state)
{
	uint8_t cnt = 0;
	
	if(state == DISABLE) 
		FLASH_TX_BUFF[cnt ++] = W25X_PowerDown;
	else  
		FLASH_TX_BUFF[cnt ++] = W25X_ReleasePowerDown;
	
	bsp_flash_Select();  
	flash_SPI_Write(FLASH_TX_BUFF, cnt);
	bsp_flash_Unselect();  
	bsp_tim_DelayUs(3);
}

/********************************************  END OF FILE  *******************************************/


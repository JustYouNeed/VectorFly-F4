/**
  *******************************************************************************************************
  * File Name: bsp_flash.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-19
  * Brief: SPI-Flash����ģ��,ʹ��оƬW25Q128
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-19
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

/*  оƬ�ͺ�ID  */
# define W25Q80 	0XEF13 	
# define W25Q16 	0XEF14
# define W25Q32 	0XEF15
# define W25Q64 	0XEF16
# define W25Q128	0XEF17


/*  �Ĵ�������ָ��  */
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

/*  FLASH��С16M  */
# define FLASH_SIZE			1024 * 1024 * 16		

/*  ����FlashƬѡ����  */
# define FLASH_CS_PORT		GPIOA
# define FLASH_CS_PIN			GPIO_Pin_15

/*  FLASH��SPIͨ��  */
# define FLASH_SPI				SPI1

/*  ��������С  */
# define FLASH_BUFF_LEN		4096		

/*  �Ƿ���DMA����  */
# define FLASH_USE_DMA		0u

/*  �Ƿ���DMA�ж�  */
# define FLASH_DMA_IRQN		0u

/*  SPI1���������ַ  */
# define SPI1_DR_ADDR    (uint32_t)0x4001300C

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
uint16_t FLASH_TYPE = W25Q128;					/*  FLASH�ͺ�,�����ж��Ƿ��ʼ���ɹ�  */
uint8_t *FLASH_TX_BUFF;	/*  ���ͻ�����  */
uint8_t *FLASH_RX_BUFF;	/*  ���ջ�����  */

/*
*********************************************************************************************************
*                          bsp_flash_Select                
*
* Description: ѡ��оƬ
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
* Description: ȡ��ѡ��оƬ
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
* Description: ��ʼ��FlashоƬƬѡ����
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
	
	/*  ʹ��DMAʱ��  */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                          /*  DMAͨ��3  */
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                /*  �������ַ  */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)FLASH_RX_BUFF;        /*  ���ջ�������ַ  */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 /*  ���������赽�ڴ�  */
	DMA_InitStructure.DMA_BufferSize = FLASH_BUFF_LEN;                      /*  �������ݳ���  */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        /*  �����ַ������  */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 /*  �ڴ��ַ����  */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         /*  ���ݴ�СΪ8λ  */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /*  ��������Ϊ8λ  */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           /*  ������ѭ��ģʽ  */
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   /*  ���ȼ��е�  */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  /*  ����ȡFIFO  */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           /*  ��Ч  */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             /*  ���δ���  */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     /*  ���δ���  */
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	
	/* DMA TX Config */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)FLASH_TX_BUFF;        /*  ���ͻ�������ַ  */
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 /*  �������ڴ浽����  */
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);
	
# if FLASH_DMA_IRQN > 0u
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;		/*  �����ж�  */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;		/*  �����ж�  */
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
* Description: DMA2������2���жϺ���
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ������ֻ����SPI1DMA��������ж�
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
* Description: DMA2������3���жϺ���
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ������ֻ����SPI1DMA��������ж�
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
* Description: ��ʼ��FlashоƬ��ʹ�õ�SPI�ӿ�
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
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//PB3~5���ù������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //PB3����Ϊ SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1); //PB4����Ϊ SPI1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1); //PB5����Ϊ SPI1

 	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//��λSPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//ֹͣ��λSPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(FLASH_SPI, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

# if FLASH_USE_DMA > 0u
	SPI_I2S_DMACmd(FLASH_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(FLASH_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
	spi_DMAConfig();
# endif
 
	SPI_Cmd(FLASH_SPI, ENABLE); /*  ʹ��SPI����  */
	SPI_I2S_ClearITPendingBit(FLASH_SPI, SPI_I2S_IT_RXNE);		/*  ������ձ�־λ  */
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
	/*  ����DMA  */
	DMA_Cmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream3, ENABLE);
	
# if FLASH_DMA_IRQN < 1u
	/*  �ȴ��������  */
	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	
	/*  �ر�DMAͨ��  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);	
	
	/*  �����־  */
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
# endif
}
/*
*********************************************************************************************************
*                            flash_SPI_ReadWrite              
*
* Description: ͨ��SPIͬʱ��дFlash,��д���ͬʱ��������
*             
* Arguments  : 1> rx_buff: ���ջ�����
*							 2> tx_buff: ���ͻ�����
*							 3> len: Ҫ���ͺͽ��յ����ݳ���
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
static void flash_SPI_ReadWrite(uint8_t *rx_buff, uint8_t *tx_buff, uint16_t len)
{
	/*  �����ô����ֽ���֮ǰ��Ҫ�ȹر�DMA  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)len);	/*  ����Ҫ������ֽ���  */
	DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)len);
	
	/*  ���ͺͽ��յ����ݵ�ַ��Ҫ����  */
	DMA2_Stream2->CR |= (1 << 10);
	DMA2_Stream3->CR |= (1 << 10);
	
	/*  ���÷��ͺͽ��յ��ڴ��ַ  */
	DMA2_Stream2->M0AR = (uint32_t)rx_buff;
	DMA2_Stream3->M0AR = (uint32_t)tx_buff;
	
	(void)FLASH_SPI->DR;	/*  ���SPI���ݼĴ���  */
	
	/*  �ȴ�������Ϊ��  */
	while(SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
		
	flash_SPI_DMAStart();		/*  ����DMA����  */
}

/*
*********************************************************************************************************
*                          flash_SPI_Write                
*
* Description: ͨ��SPI��ȡFlash
*             
* Arguments  : 1> tx_buff: �������ݻ�����
*							 2> len: Ҫ���͵����ݳ���
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void flash_SPI_Write(uint8_t *tx_buff, uint16_t len)
{
	uint8_t temp[1];
	
	/*  �����ô����ֽ���֮ǰ��Ҫ�ȹر�DMA  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	
	DMA_SetCurrDataCounter(DMA2_Stream2, 1);	/*  ����Ҫ������ֽ���  */
	DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)len);
	
	/*  ���͵���Ҫ����,���յĲ���Ҫ,��Ϊ���ﲻ����  */
	DMA2_Stream2->CR &= ~(1 << 10);
	
	/*  ���÷��ͺͽ��յ��ڴ��ַ  */
	DMA2_Stream2->M0AR = (uint32_t)temp;
	DMA2_Stream3->M0AR = (uint32_t)tx_buff;
	
	(void)FLASH_SPI->DR;	/*  ���SPI���ݼĴ���  */
	
	/*  �ȴ�������Ϊ��  */
	while(SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
	
	flash_SPI_DMAStart();		/*  ����DMA����  */
}

/*
*********************************************************************************************************
*                             flash_SPI_Read             
*
* Description: ͨ��SPI��ȡFlash
*             
* Arguments  : 1> rx_buff: ��ȡ������
*							 2> len: Ҫ��ȡ�����ݳ���
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void flash_SPI_Read(uint8_t *rx_buff, uint16_t len)
{
	uint8_t temp[1];
	
	/*  �����ô����ֽ���֮ǰ��Ҫ�ȹر�DMA  */
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);
	
	DMA_SetCurrDataCounter(DMA2_Stream2, (uint16_t)len);	/*  ����Ҫ������ֽ���  */
	DMA_SetCurrDataCounter(DMA2_Stream3, (uint16_t)len);
	
	/*  ���յ���Ҫ����,���͵Ĳ���Ҫ  */
	DMA2_Stream3->CR &= ~(1 << 10);
	
	/*  ���÷��ͺͽ��յ��ڴ��ַ  */
	DMA2_Stream2->M0AR = (uint32_t)rx_buff;
	DMA2_Stream3->M0AR = (uint32_t)temp;
	
	(void)FLASH_SPI->DR;	/*  ���SPI���ݼĴ���  */
	
	/*  �ȴ�������Ϊ��  */
	while(SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
	
	flash_SPI_DMAStart();		/*  ����DMA����  */
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
	SPI_I2S_SendData(FLASH_SPI, byte); //ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(FLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);  
	return SPI_I2S_ReceiveData(FLASH_SPI); //����ͨ��SPIx������յ�����
}
/*
*********************************************************************************************************
*                            flash_SPI_ReadWrite              
*
* Description: ͨ��SPIͬʱ��дFlash,��д���ͬʱ��������
*             
* Arguments  : 1> rx_buff: ���ջ�����
*							 2> tx_buff: ���ͻ�����
*							 3> len: Ҫ���ͺͽ��յ����ݳ���
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
* Description: ͨ��SPI��ȡFlash
*             
* Arguments  : 1> tx_buff: �������ݻ�����
*							 2> len: Ҫ���͵����ݳ���
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
* Description: ͨ��SPI��ȡFlash
*             
* Arguments  : 1> rx_buff: ��ȡ������
*							 2> len: Ҫ��ȡ�����ݳ���
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
* Description: ��ȡFLASH��ID
*             
* Arguments  : None.
*
* Reutrn     : ��ȡ����ID
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
* Description: ��ȡFlash���ڴ��С
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
* Description: ��ȡFlash��״̬�Ĵ���
*             
* Arguments  : None.
*
* Reutrn     : ��ȡ�ĵ�Flash״̬�Ĵ�����ֵ
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
* Description: дFlash״̬�Ĵ���
*             
* Arguments  : 1> sr: ״̬�Ĵ���ֵ
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
* Description: Flashдʹ�ܺ���
*             
* Arguments  : 1> state: ENABLEдʹ��, DISABLE:дʧ��
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
* Description: FlashоƬҳд����
*             
* Arguments  : 1> buff: ���ݻ�����
*							 2> addr: Ҫд��ĵ�ַ
*							 3> len: ���ݳ���
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
	
	bsp_flash_WriteCmd(ENABLE);		/*  дʹ��  */
	
	bsp_flash_Select();
	flash_SPI_Write(FLASH_TX_BUFF, cnt);
	flash_SPI_Write(buff, len);
	bsp_flash_Unselect();
	
	bsp_flash_WaitBusy();			/*  �ȴ�д�����  */
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
	pageremain=256-addr%256; //��ҳʣ����ֽ���		 	    
	if(len<=pageremain)pageremain=len;//������256���ֽ�
	while(1)
	{	   
		bsp_flash_WritePage(buff,addr,pageremain);
		if(len==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			buff+=pageremain;
			addr+=pageremain;	

			len-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(len>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=len; 	  //����256���ֽ���
		}
	}
}


/*
*********************************************************************************************************
*                        bsp_flash_Read                  
*
* Description: ��ȡFlash����
*             
* Arguments  : 1> buff: ���ݻ�����
*							 2> addr: Ҫ��ȡ�ĵ�ַ
*							 3> len: Ҫ��ȡ�����ݳ���
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
* Description: Flashд���ݺ���
*             
* Arguments  : 1> buff: ���ݻ�����
*							 2> addr: Ҫд��ĵ�ַ
*							 3> len: ���ݳ���
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
	
 	secpos = addr/4096;//������ַ  
	secoff = addr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
	
 	if(len <= secremain) secremain = len;//������4096���ֽ�
	while(1) 
	{	
		bsp_flash_Read(FLASH_TX_BUFF, secpos*4096, 4096);//������������������
		for(i = 0; i < secremain; i++)//У������
		{
			if(FLASH_TX_BUFF[secoff+i] != 0XFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			bsp_flash_EraseSector(secpos);//�����������
			for(i = 0; i < secremain; i++)	   //����
			{
				FLASH_TX_BUFF[i+secoff] = *(buff + i);	  
			}
			bsp_flash_WriteNoCheck(FLASH_TX_BUFF, secpos*4096,4096);//д����������  

		}else bsp_flash_WriteNoCheck(buff, addr, secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(len == secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

			buff += secremain;  //ָ��ƫ��
			addr += secremain;//д��ַƫ��	   
			len -= secremain;				//�ֽ����ݼ�
			if(len > 4096)secremain = 4096;	//��һ����������д����
			else secremain = len;			//��һ����������д����
		}	 
	}
}


/*
*********************************************************************************************************
*                         bsp_flash_EraseChip                 
*
* Description: ȫƬ����Flash
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
	bsp_flash_Select();    	//ʹ������   
	flash_SPI_Write(FLASH_TX_BUFF, cnt);
	bsp_flash_Unselect();                           //ȡ��Ƭѡ     	      
	bsp_flash_WaitBusy();   				   //�ȴ�оƬ��������
}


/*
*********************************************************************************************************
*                     bsp_flash_EraseSector                     
*
* Description: ���������
*             
* Arguments  : 1> dtsAddr: Ҫ�����ĵ�ַ
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
	bsp_flash_Select();                              //ʹ������   
	flash_SPI_Write(FLASH_TX_BUFF, cnt); 
	bsp_flash_Unselect();                            //ȡ��Ƭѡ     	      
	bsp_flash_WaitBusy();   				   //�ȴ��������
}


/*
*********************************************************************************************************
*                        bsp_flash_WaitBusy                  
*
* Description: �ȴ�Flash��������
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
* Description: Flash�ϵ纯��
*             
* Arguments  : 1> state: ENABLE:�ϵ�, DISABLE: ����
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


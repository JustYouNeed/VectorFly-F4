/**
  *******************************************************************************************************
  * File Name: bsp_uarth
  * Author: Vector
  * Version: V.0.0
  * Date: 2018-7-29
  * Brief: 本文件提供了飞控底层串口驱动代码
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-7-29
	*			Mod: 建立文件,添加基本函数
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# define UART_LINK_BAUDRATE					961200
# define UART_LINK_DATA_QUEUE_LEN		128
# define UART_LINK_DATA_TIMEOUT_MS	1000
# define UART_LINK_DATA_TIMEOUT_TICKS	(UART_LINK_DATA_TIMEOUT_MS / portTICK_RATE_MS)


/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static bool	isInit = false;
static bool isUartDmaInit = false;
static bool isDmaPaused = true;

static xSemaphoreHandle	uartWaitSendDone = NULL;
static xSemaphoreHandle uartBusy = NULL;
static xQueueHandle uartRxDataQueue = NULL;

static uint8_t *uartSendData;				/*  串口中断发送数据缓存区  */
static uint8_t uartSendDataIndex;		/*  中断发送数据计数  */
static uint8_t uartSendDataSize;		/*  需要中断发送的数据总数  */

static uint8_t uartDmaBuff[64];			/*  中断DMA缓存区  */
static DMA_InitTypeDef DMA_InitStructure;

/*  以下两个变量用于DMA的暂时/开始  */
static uint32_t initDMADataCount;			/*  DMA初始发送数据大小  */
static uint32_t reamainingDMADataCount;	/*  DMA剩下的要发送的数据  */


/*
*********************************************************************************************************
*                          bsp_uart_GPIOConfig                
*
* Description: 初始化串口GPIO引脚
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_uart_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	bsp_rcc_GPIOClcokCmd(GPIOB);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*                       bsp_uart_DMAConfig                   
*
* Description: 初始化串口DMA功能
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_uart_DMAConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/*  配置串口发送DMA  */
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uartDmaBuff;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		
	isUartDmaInit = true;
}

/*
*********************************************************************************************************
*                     bsp_uart_UARTConfig                     
*
* Description: 初始化串口相关参数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_uart_UARTConfig(void)
{
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitStructure.USART_BaudRate = UART_LINK_BAUDRATE;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);	/*  先清除接收中断标志位  */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); /*  使能接收中断  */
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
}


/*
*********************************************************************************************************
*                    bsp_uart_NVICConfig                      
*
* Description: 初始化串口中断功能
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_uart_NVICConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
		
	/*  串口中断收发  */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*  串口DMA中断  */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*                         bsp_uart_MsgConfig                 
*
* Description: 初始化串口系统通信相关变量
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_uart_MsgConfig(void)
{
	uartWaitSendDone = xSemaphoreCreateBinary();	/*  等待串口发送完成二值信号量  */
	uartBusy = xSemaphoreCreateBinary();					/*  串口忙二值信号量  */
	xSemaphoreGive(uartBusy);											/*  先释放一次忙信号,串口处于空闲状态  */
	
	/*  创建数据接收队列  */
	uartRxDataQueue = xQueueCreate(UART_LINK_DATA_QUEUE_LEN, sizeof(uint8_t));
}


/*
*********************************************************************************************************
*                       bsp_uart_Init                   
*
* Description: 初始化串口功能
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_Init(void)
{
	if(isInit) return ;
	bsp_uart_MsgConfig();
	bsp_uart_GPIOConfig();
	bsp_uart_UARTConfig();
	bsp_uart_DMAConfig();
	bsp_uart_NVICConfig();
	
	isInit = true;
}


/*
*********************************************************************************************************
*                      bsp_uart_GetDataWithTimeout                    
*
* Description: 从串口数据接上消息队列中读取一个字节的数据,有超时处理
*             
* Arguments  : 1> byte: 数据接收指针
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
bool bsp_uart_GetDataWithTimeout(uint8_t *byte)
{
	if(xQueueReceive(uartRxDataQueue, byte, UART_LINK_DATA_TIMEOUT_TICKS) == pdTRUE)
	{
		return true;
	}
	
	*byte = 0;
	return false;
}


/*
*********************************************************************************************************
*                         bsp_uart_SendData                 
*
* Description: 串口发送原始数据
*             
* Arguments  : 1.buff: 要发送的数据缓存区
*							 2.len: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_SendData(uint8_t *buff, uint32_t len)
{
	uint32_t i = 0;
	
	if(!isInit) return ;
	
	for(i = 0; i < len; i++)
	{
		while(!(USART1->SR & USART_FLAG_TXE));
		USART1->DR = (buff[i] & 0x00ff);
	}
}

/*
*********************************************************************************************************
*                      bsp_uart_SendDataIsrBlocking                    
*
* Description: 串口中断发送数据
*             
* Arguments  : 1.buff: 要发送的数据缓存区
*							 2.len: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : 该函数会阻塞直到串口空闲
*********************************************************************************************************
*/
void bsp_uart_SendDataIsrBlocking(uint8_t *buff, uint32_t len)
{
	/*  获取串口空闲信号量  */
	xSemaphoreTake(uartBusy, portMAX_DELAY);
	
	uartSendData = buff;
	uartSendDataSize = len;
	uartSendDataIndex = 1;
	bsp_uart_SendData(&buff[0], 1);
	
	/*  开启串口发送空闲中断  */
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	/*  等待串口发送完成  */
	xSemaphoreTake(uartWaitSendDone, portMAX_DELAY);
	uartSendData  = 0;
	
	/*  释放串口忙信号量  */
	xSemaphoreGive(uartBusy);
}

/*
*********************************************************************************************************
*                        bsp_uart_PutChar                  
*
* Description: 发送一个字符到串口
*             
* Arguments  : 1.c: 要发送的字符
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
int  bsp_uart_PutChar(int c)
{
	bsp_uart_SendData((uint8_t *)&c, 1);
	return (uint8_t)c;
}

/*
*********************************************************************************************************
*                        bsp_uart_SendDataDmaBlocking                  
*
* Description: 通过DMA发送原始数据
*             
* Arguments  : 1.buff: 要发送的数据缓存区
*							 2.len: 要发送的数据长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_SendDataDmaBlocking(uint8_t *buff, uint32_t len)
{
	if(!isUartDmaInit) return ;
	
	/*  占用串口  */
	xSemaphoreTake(uartBusy, portMAX_DELAY);	
	
	/*  等待DMA空闲  */
	while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
	
	memcpy(uartDmaBuff, buff, len);		/*  复制数据到发送缓存区  */						
	DMA_InitStructure.DMA_BufferSize = len;			
	initDMADataCount = len;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);			/*  重新初始化DMA  */
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	/*  开启DMA发送完成中断  */	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	/*  开启串口DMA发送  */	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	DMA_Cmd(DMA2_Stream7, ENABLE);					/*  使能DMA2数据流7  */
	xSemaphoreTake(uartWaitSendDone, portMAX_DELAY);
	xSemaphoreGive(uartBusy);
}



/*
*********************************************************************************************************
*                       bsp_uart_Isr                   
*
* Description: 串口中断处理函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_Isr(void)
{
	uint8_t rxData = 0;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/*  接收到数据就发送到消息队列  */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rxData = USART_ReceiveData(USART1);
		usb_printf(rxData);
		xQueueSendFromISR(uartRxDataQueue, &rxData, &xHigherPriorityTaskWoken);
	}
	
	/*  串口空闲中断  */
	else if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		/*  还有数据需要发送的话就一直发送  */
		if(uartSendData && (uartSendDataIndex < uartSendDataSize))
		{
			USART_SendData(UART_LINK_COM, uartSendData[uartSendDataIndex ++] & 0x00ff);
		}	
		else		/*  没有数据需要发送了就关闭发送空闲中断,同时释放串口发送完成信号量  */
		{
			USART_ITConfig(UART_LINK_COM, USART_IT_TXE, DISABLE);
			xSemaphoreGiveFromISR(uartWaitSendDone, &xHigherPriorityTaskWoken);
		}
	}
}

/*
*********************************************************************************************************
*                            bsp_uart_DMAPause              
*
* Description: 暂停串口DMA传输
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void bsp_uart_DMAPause(void)
{
	if(DMA_GetCmdStatus(DMA2_Stream7) == ENABLE)
	{
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);	/*关闭DMA传输完成中断*/	
		DMA_Cmd(DMA2_Stream7, DISABLE);
		while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
		reamainingDMADataCount = DMA_GetCurrDataCounter(DMA2_Stream7);
		isDmaPaused = true;
	}
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
static void bsp_uart_DMAResume(void)
{
	if (isDmaPaused)
	{
		DMA_SetCurrDataCounter(DMA2_Stream7, reamainingDMADataCount);	/*更新DMA计数器*/
		DMA2_Stream7->M0AR = (u32)&uartDmaBuff[initDMADataCount - reamainingDMADataCount];/*更新内存读取地址*/
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	/*开启DMA传输完成中断*/	
		USART_ClearFlag(USART1, USART_FLAG_TC);	/* 清除传输完成中断标志位 */
		DMA_Cmd(DMA2_Stream7, ENABLE);	/* 使能DMA USART TX数据流 */
		isDmaPaused = false;
	}
}

/*
*********************************************************************************************************
*                      bsp_uart_DMAIsr                    
*
* Description: 串口DMA中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_DMAIsr(void)
{
	portBASE_TYPE	xHigherPriorityTaskWoken = pdFALSE;
	
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	USART_DMACmd(UART_LINK_COM, USART_DMAReq_Tx, DISABLE);
	DMA_Cmd(DMA2_Stream7, DISABLE);
	
	xSemaphoreGiveFromISR(uartWaitSendDone, &xHigherPriorityTaskWoken);
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
void __attribute__((used)) USART1_IRQHandler(void)
{
	bsp_uart_Isr();
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
void __attribute__((used)) DMA2_Stream7_IRQHandler(void)
{
	bsp_uart_DMAIsr();
}
/********************************************  END OF FILE  *******************************************/


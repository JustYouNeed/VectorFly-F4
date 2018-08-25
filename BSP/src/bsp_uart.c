/**
  *******************************************************************************************************
  * File Name: bsp_uarth
  * Author: Vector
  * Version: V.0.0
  * Date: 2018-7-29
  * Brief: ���ļ��ṩ�˷ɿصײ㴮����������
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-7-29
	*			Mod: �����ļ�,��ӻ�������
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

static uint8_t *uartSendData;				/*  �����жϷ������ݻ�����  */
static uint8_t uartSendDataIndex;		/*  �жϷ������ݼ���  */
static uint8_t uartSendDataSize;		/*  ��Ҫ�жϷ��͵���������  */

static uint8_t uartDmaBuff[64];			/*  �ж�DMA������  */
static DMA_InitTypeDef DMA_InitStructure;

/*  ����������������DMA����ʱ/��ʼ  */
static uint32_t initDMADataCount;			/*  DMA��ʼ�������ݴ�С  */
static uint32_t reamainingDMADataCount;	/*  DMAʣ�µ�Ҫ���͵�����  */


/*
*********************************************************************************************************
*                          bsp_uart_GPIOConfig                
*
* Description: ��ʼ������GPIO����
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
* Description: ��ʼ������DMA����
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
	
	/*  ���ô��ڷ���DMA  */
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
* Description: ��ʼ��������ز���
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
	
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);	/*  ����������жϱ�־λ  */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); /*  ʹ�ܽ����ж�  */
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
}


/*
*********************************************************************************************************
*                    bsp_uart_NVICConfig                      
*
* Description: ��ʼ�������жϹ���
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
		
	/*  �����ж��շ�  */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/*  ����DMA�ж�  */
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
* Description: ��ʼ������ϵͳͨ����ر���
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
	uartWaitSendDone = xSemaphoreCreateBinary();	/*  �ȴ����ڷ�����ɶ�ֵ�ź���  */
	uartBusy = xSemaphoreCreateBinary();					/*  ����æ��ֵ�ź���  */
	xSemaphoreGive(uartBusy);											/*  ���ͷ�һ��æ�ź�,���ڴ��ڿ���״̬  */
	
	/*  �������ݽ��ն���  */
	uartRxDataQueue = xQueueCreate(UART_LINK_DATA_QUEUE_LEN, sizeof(uint8_t));
}


/*
*********************************************************************************************************
*                       bsp_uart_Init                   
*
* Description: ��ʼ�����ڹ���
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
* Description: �Ӵ������ݽ�����Ϣ�����ж�ȡһ���ֽڵ�����,�г�ʱ����
*             
* Arguments  : 1> byte: ���ݽ���ָ��
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
* Description: ���ڷ���ԭʼ����
*             
* Arguments  : 1.buff: Ҫ���͵����ݻ�����
*							 2.len: Ҫ���͵����ݳ���
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
* Description: �����жϷ�������
*             
* Arguments  : 1.buff: Ҫ���͵����ݻ�����
*							 2.len: Ҫ���͵����ݳ���
*
* Reutrn     : None.
*
* Note(s)    : �ú���������ֱ�����ڿ���
*********************************************************************************************************
*/
void bsp_uart_SendDataIsrBlocking(uint8_t *buff, uint32_t len)
{
	/*  ��ȡ���ڿ����ź���  */
	xSemaphoreTake(uartBusy, portMAX_DELAY);
	
	uartSendData = buff;
	uartSendDataSize = len;
	uartSendDataIndex = 1;
	bsp_uart_SendData(&buff[0], 1);
	
	/*  �������ڷ��Ϳ����ж�  */
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	/*  �ȴ����ڷ������  */
	xSemaphoreTake(uartWaitSendDone, portMAX_DELAY);
	uartSendData  = 0;
	
	/*  �ͷŴ���æ�ź���  */
	xSemaphoreGive(uartBusy);
}

/*
*********************************************************************************************************
*                        bsp_uart_PutChar                  
*
* Description: ����һ���ַ�������
*             
* Arguments  : 1.c: Ҫ���͵��ַ�
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
* Description: ͨ��DMA����ԭʼ����
*             
* Arguments  : 1.buff: Ҫ���͵����ݻ�����
*							 2.len: Ҫ���͵����ݳ���
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_uart_SendDataDmaBlocking(uint8_t *buff, uint32_t len)
{
	if(!isUartDmaInit) return ;
	
	/*  ռ�ô���  */
	xSemaphoreTake(uartBusy, portMAX_DELAY);	
	
	/*  �ȴ�DMA����  */
	while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
	
	memcpy(uartDmaBuff, buff, len);		/*  �������ݵ����ͻ�����  */						
	DMA_InitStructure.DMA_BufferSize = len;			
	initDMADataCount = len;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);			/*  ���³�ʼ��DMA  */
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	/*  ����DMA��������ж�  */	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	/*  ��������DMA����  */	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	DMA_Cmd(DMA2_Stream7, ENABLE);					/*  ʹ��DMA2������7  */
	xSemaphoreTake(uartWaitSendDone, portMAX_DELAY);
	xSemaphoreGive(uartBusy);
}



/*
*********************************************************************************************************
*                       bsp_uart_Isr                   
*
* Description: �����жϴ�����
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
	
	/*  ���յ����ݾͷ��͵���Ϣ����  */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rxData = USART_ReceiveData(USART1);
		usb_printf(rxData);
		xQueueSendFromISR(uartRxDataQueue, &rxData, &xHigherPriorityTaskWoken);
	}
	
	/*  ���ڿ����ж�  */
	else if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		/*  ����������Ҫ���͵Ļ���һֱ����  */
		if(uartSendData && (uartSendDataIndex < uartSendDataSize))
		{
			USART_SendData(UART_LINK_COM, uartSendData[uartSendDataIndex ++] & 0x00ff);
		}	
		else		/*  û��������Ҫ�����˾͹رշ��Ϳ����ж�,ͬʱ�ͷŴ��ڷ�������ź���  */
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
* Description: ��ͣ����DMA����
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
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);	/*�ر�DMA��������ж�*/	
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
		DMA_SetCurrDataCounter(DMA2_Stream7, reamainingDMADataCount);	/*����DMA������*/
		DMA2_Stream7->M0AR = (u32)&uartDmaBuff[initDMADataCount - reamainingDMADataCount];/*�����ڴ��ȡ��ַ*/
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	/*����DMA��������ж�*/	
		USART_ClearFlag(USART1, USART_FLAG_TC);	/* �����������жϱ�־λ */
		DMA_Cmd(DMA2_Stream7, ENABLE);	/* ʹ��DMA USART TX������ */
		isDmaPaused = false;
	}
}

/*
*********************************************************************************************************
*                      bsp_uart_DMAIsr                    
*
* Description: ����DMA�жϺ���
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


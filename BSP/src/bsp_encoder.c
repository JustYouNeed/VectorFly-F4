/**
  *******************************************************************************************************
  * File Name: bsp_enconder.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-4-5
  * Brief: ±¾ÎÄ¼þÌá¹©ÁËÓÐ¹Øµç»ú±àÂëÆ÷µÄÏà¹Ø²Ù×÷º¯Êý
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-4-5
	*			Mod: ½¨Á¢ÎÄ¼þ
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# define ENCODER_TIM_PERIOD (u16)(65535)   //²»¿É´óÓÚ65535 ÒòÎªF103µÄ¶¨Ê±Æ÷ÊÇ16Î»µÄ¡

/*
*********************************************************************************************************
*                       Encoder_TIM3Config                   
*
* Description: ³õÊ¼»¯¶¨Ê±Æ÷3Îª±àÂëÆ÷Ä£Ê½,¶ÔÓ¦×ó±ßµç»ú±àÂëÆ÷
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ±¾ÎÄ¼þ Ë½ÓÐº¯Êý
*********************************************************************************************************
*/
static void Encoder_TIM3Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//Ê¹ÄÜ¶¨Ê±Æ÷3µÄÊ±ÖÓ
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//Ê¹ÄÜPA¶Ë¿ÚÊ±ÖÓ
	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//¶Ë¿ÚÅäÖÃ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //¸¡¿ÕÊäÈë
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOA
	
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ô¤·ÖÆµÆ÷ 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//Ñ¡ÔñÊ±ÖÓ·ÖÆµ£º²»·ÖÆµ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIMÏòÉÏ¼ÆÊý  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//Ê¹ÓÃ±àÂëÆ÷Ä£Ê½3
	
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);//Çå³ýTIMµÄ¸üÐÂ±êÖ¾Î»
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
  TIM_SetCounter(TIM3,0);
  TIM_Cmd(TIM3, ENABLE); 

}
/*
*********************************************************************************************************
*                     Encoder_TIM4Config                     
*
* Description: ³õÊ¼»¯¶¨Ê±Æ÷4Îª±àÂëÆ÷,ÓÒ±ßµç»úµÄ±àÂëÆ÷
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : ±¾ÎÄ¼þË½ÓÐº¯Êý
*********************************************************************************************************
*/
static void Encoder_TIM4Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//Ê¹ÄÜ¶¨Ê±Æ÷4µÄÊ±ÖÓ
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//Ê¹ÄÜPB¶Ë¿ÚÊ±ÖÓ
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//¶Ë¿ÚÅäÖÃ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //¸¡¿ÕÊäÈë
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ô¤·ÖÆµÆ÷ 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//Ñ¡ÔñÊ±ÖÓ·ÖÆµ£º²»·ÖÆµ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIMÏòÉÏ¼ÆÊý  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//Ê¹ÓÃ±àÂëÆ÷Ä£Ê½3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//Çå³ýTIMµÄ¸üÐÂ±êÖ¾Î»
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
}

/*
*********************************************************************************************************
*                        bsp_encoder_Config                  
*
* Description: ³õÊ¼»¯±àÂëÆ÷
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_Config(void)
{
	Encoder_TIM3Config();
	Encoder_TIM4Config();
}




/*
*********************************************************************************************************
*                            bsp_encoder_ReadValue              
*
* Description: ¶ÁÈ¡µç»ú±àÂëÆ÷Öµ
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_ReadValue(void)
{
	/*  ¶ÁÈ¡±àÂëÆ÷Öµ  */
	
	/*  ¼ÆÊýÆ÷ÇåÁã  */
	TIM3->CNT = 0;
	TIM4->CNT = 0;
}


/**************************************************************************
º¯Êý¹¦ÄÜ£ºTIM4ÖÐ¶Ï·þÎñº¯Êý
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//Òç³öÖÐ¶Ï
	{    				
	
	}		
	TIM4->SR&=~(1<<0);//Çå³ýÖÐ¶Ï±êÖ¾Î» 	 	
}


/**************************************************************************
º¯Êý¹¦ÄÜ£ºTIM2ÖÐ¶Ï·þÎñº¯Êý
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//Òç³öÖÐ¶Ï
	{    				
		
	}				  
	TIM3->SR&=~(1<<0);//Çå³ýÖÐ¶Ï±êÖ¾Î» 	    	
}

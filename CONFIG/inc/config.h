/**
  *******************************************************************************************************
  * File Name: config.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-8-23
  * Brief: Vector配置文件
  *******************************************************************************************************
  * History
  *		Author: Vector
	*		Date: 2018-8-23
	*		Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
# ifndef __CONFIG_H
# define __CONFIG_H

/*  串口部分配置  */
# define UART_LINK_COM				USART1
# define UART_LINK_PORT				GPIOB
# define UART_LINK_RX_PIN			GPIO_Pin_6
# define UART_LINK_TX_PIN			GPIO_Pin_7
# define UART_LINK_BAUDRATE		961200

/*  MPU9250引脚配置部分  */
# define MPU_SDA_PORT			GPIOB
# define MPU_SDA_PIN			GPIO_Pin_8

# define MPU_SCL_PORT			GPIOB
# define MPU_SCL_PIN			GPIO_Pin_9

# define MPU_INT_PORT			GPIOB
# define MPU_INT_PIN			GPIO_Pin_5

# endif


/********************************************  END OF FILE  *******************************************/



/**
  *******************************************************************************************************
  * File Name: bsp.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-19
  * Brief: 极级支持包头文件
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-19
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */

# ifndef __BSP_H
# define __BSP_H

/*  定义使用的开发板芯片  */
# if defined (STM32F10X_HD)|| defined (STM32F10X_MD)|| defined (STM32F10X_LD)
	# define VECTOR_F1		1u
# elif defined (STM32F40_41xxx) || defined (STM32F401xx)
	# define VECTOR_F4		1u
# endif

# define USE_UCOSIII 0u		/*  使用UCOSIII操作系统  */
# define USE_FREERTOS	1u

/*********************************************  系统头文件  ** ****************************************/
# include "stdio.h"
# include "string.h"
# include <stdarg.h>
# include "math.h"
# include "bsp_com.h"
# include "stm32f4xx.h"
# include "config.h"
/*********************************************  分割线  ***********************************************/

# if USE_FREERTOS > 0u
	# include "FreeRTOS.h"
	# include "task.h"
	# include "queue.h"
	# include "semphr.h"
	# include "event_groups.h"
	# include "FreeRTOS_Task.h"
# endif

# include "usbd_cdc_core.h"
# include "usbd_usr.h"
# include "usb_conf.h"
# include "usbd_desc.h"
# include "usbd_cdc_core.h"
# include "usbd_cdc_vcp.h"
# include "usbd_usr.h"

# include "i2cdrv.h"
# include "pid.h"
# include "sort.h"
# include "filter.h"

/***************************************   板级支持头文件   *******************************************/
# include "bsp_rcc.h"
# include "bsp_timer.h"
# include "bsp_sram.h"
# include "bsp_malloc.h"
# include "bsp_led.h"
# include "bsp_uart.h"
# include "bsp_led.h"
# include "bsp_key.h"
# include "bsp_encoder.h"
# include "bsp_motor.h"
# include "bsp_servo.h"
# include "bsp_rocker.h"
# include "bsp_beep.h"
# include "bsp_lcd.h"
# include "bsp_motor.h"
# include "bsp_servo.h"
# include "bsp_flash.h"
# include "bsp_ee.h"
# include "bsp_mpu.h"
# include "bsp_magnet.h"
//# include "bsp_sdcard.h"
//# include "bsp_lan8720.h"
/*********************************************  分割线  **********************************************/

void bsp_Config(void);
void bsp_IdleTask(void);
# endif


/********************************************  END OF FILE  *******************************************/


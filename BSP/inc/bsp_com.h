/**
  *******************************************************************************************************
  * File Name: common.h
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-5-29
  * Brief: 提供一些32上的常用指令
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-5-29
	*			Mod: 建立文件
	*
	*		2.Author: Vector
	*			Date: 2018-6-23
	*			Mod: 1.统一F1和F4的位操作宏定义
	*					 2.增加引脚输出高低宏定义,采用操作寄存器方式
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_COMMON_H
# define __BSP_COMMON_H


/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

#ifndef __cplusplus
	#ifndef bool
		typedef enum {FALSE = 0, TRUE = !FALSE} bool;
	#else
		#define FALSE  0
		#define TRUE	!FALSE
	#endif
#endif
		
# ifndef null
# define null (void*)0
# endif

# ifndef NULL
# define NULL (void *)0
# endif

# ifndef TRUE	
	# define TRUE 1
# endif

# ifndef true
	# define true 1
# endif
	
# ifndef FALSE
	# define FALSE	0
# endif
	
# ifndef false
	# define false 0
# endif
	

# ifdef VECTOR_F1
	# define BIT_OFFSET	12
	# define PIN_OUT_HIGH(PORT, PIN)		PORT->BSRR = PIN
	# define PIN_OUT_LOW(PORT, PIN)		PORT->BRR = PIN
	# define PIN_TOGGLE(PORT, PIN)			PORT->ODR ^= PIN
# elif defined (VECTOR_F4)
	#define BIT_OFFSET	20
	# define PIN_OUT_HIGH(PORT, PIN)		PORT->BSRRL = PIN
	# define PIN_OUT_LOW(PORT, PIN)			PORT->BSRRH = PIN
	# define PIN_TOGGLE(PORT, PIN)			PORT->ODR ^= PIN
	# define PIN_READ(PORT, PIN)				((PORT->IDR & PIN) != 0)
# endif

//	位带操作,实现51类似的GPIO控制功能
//	具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//  IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE + BIT_OFFSET) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE + BIT_OFFSET) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE + BIT_OFFSET) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE + BIT_OFFSET) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE + BIT_OFFSET) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE + BIT_OFFSET) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE + BIT_OFFSET) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE + BIT_OFFSET - 4) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE + BIT_OFFSET - 4) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE + BIT_OFFSET - 4) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE + BIT_OFFSET - 4) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE + BIT_OFFSET - 4) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE + BIT_OFFSET - 4) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE + BIT_OFFSET - 4) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

# endif

	
/********************************************  END OF FILE  *******************************************/


/**
  *******************************************************************************************************
  * File Name: common.h
  * Author: Vector
  * Version: V1.1.0
  * Date: 2018-5-29
  * Brief: �ṩһЩ32�ϵĳ���ָ��
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-5-29
	*			Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-6-23
	*			Mod: 1.ͳһF1��F4��λ�����궨��
	*					 2.������������ߵͺ궨��,���ò����Ĵ�����ʽ
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_COMMON_H
# define __BSP_COMMON_H


/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

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

//	λ������,ʵ��51���Ƶ�GPIO���ƹ���
//	����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//  IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
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
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

# endif

	
/********************************************  END OF FILE  *******************************************/


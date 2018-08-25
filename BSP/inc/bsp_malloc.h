/**
  *******************************************************************************************************
  * File Name: bsp_malloc.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-21
  * Brief: �ڴ����ģ��
  *******************************************************************************************************
  * History
	*		1.Author: Vector
  *			Date: 2018-2-11
  *			Mod: �����ļ�
  *******************************************************************************************************
  */
# ifndef __BSP_MALLOC_H
# define __BSP_MALLOC_H


# define USE_SRAMIN		1u   /* �Ƿ�ʹ���ڲ�SRAM */
# define USE_SRAMEX		0u   /* �Ƿ�ʹ���ⲿSRAM */
# define USE_SRAMCCM	0u   /* �Ƿ�ʹ���ڲ�CCM�ڴ棬��F1��Ч */

/* �ڴ������� */
# define  SRAMBANK		(USE_SRAMIN + USE_SRAMEX + USE_SRAMCCM)


typedef enum{
	# if USE_SRAMIN > 0u
	SRAMIN,
	# endif
	
	# if USE_SRAMEX > 0u
	SRAMEX,
	# endif
	
	# if USE_SRAMCCM > 0u
	SRAMCCM
	# endif
}SRAM_ID;


# if USE_SRAMIN > 0u
	# define SRAMIN_BASE		
	# define SRAMIN_BLOCK_SIZE	32                  /*  �����ڴ��Ĵ�СΪ32���ֽ�  */
	# define SRAMIN_MAX_SIZE		96*1024							/*  ����ڴ�96  */	
	# define SRAMIN_TABLE_SIZE	SRAMIN_MAX_SIZE/SRAMIN_BLOCK_SIZE			/*  �ڴ�����Ĵ�С  */
# endif

# if USE_SRAMEX > 0u
	# define SRAMEX_BASE					0X68000000
	# define SRAMEX_BLOCK_SIZE		32						/*  ����ڴ��Ĵ�СΪ32�ֽ�  */
	# define SRAMEX_MAX_SIZE			960*1024			/*  ������960���ֽ��ڴ�  */	
	# define SRAMEX_TABLE_SIZE	SRAMEX_MAX_SIZE/SRAMEX_BLOCK_SIZE		/*  �ڴ�����Ĵ�С  */
# endif

# if USE_SRAMCCM > 0u
	# define SRAMCCM_BASE					0X10000000
	# define SRAMCCM_BLOCK_SIZE		32				/*  �����ڴ��Ĵ�СΪ32�ֽ�  */
	# define SRAMCCM_MAX_SIZE			60*1024		/*  ������60���ֽ�  */
	# define SRAMCCM_TABLE_SIZE	SRAMCCM_MAX_SIZE/SRAMCCM_BLOCK_SIZE			/*  �ڴ�����Ĵ�С  */
# endif


/*    */
typedef struct
{
	void (*malloc_config)(void );   	/*  ��ʼ������  */
# if USE_SRAMEX > 0u 								/*  ���ʹ���ⲿSRAM������Ҫ�ṩ�ⲿSRAM��ʼ������  */
	void (*sram_config)(void);				/*  �ⲿSRAM��ʼ������  */
# endif
	uint8_t (*persued)(uint8_t );			/*  �ڴ�ʹ����  */
	uint8_t *membase[SRAMBANK];				/*  �ڴ�ػ���ַ  */
	uint16_t *memmap[SRAMBANK];				/*  �ڴ�����  */
	uint8_t memready[SRAMBANK];				/*  �ڴ�ؾ���ָʾ  */
}MEM_TypeDef;

extern MEM_TypeDef malloc_dev;				/*    */

void bsp_mem_Set(void *s, uint8_t value, uint32_t count);		/*  �����ڴ�  */
void bsp_mem_Copy(void *des, void *src, uint32_t count);			/*  �����ڴ�  */
void bsp_mem_Config(void);																		/*  �ڴ�����ʼ��  */
uint8_t bsp_mem_GetUsed(uint8_t memx);												/*  ��ȡ�ڴ�ʹ����  */

void bsp_mem_Free(uint8_t memx, void *mem);										/*  �ڴ�����  */
void *bsp_mem_Alloc(uint8_t memx, uint32_t size);							/*  �ڴ��ͷ�  */
void *bsp_mem_Realloc(uint8_t memx, void *mem, uint32_t size);	/*  ���·����ڴ�  */
# endif

/********************************************  END OF FILE  *******************************************/


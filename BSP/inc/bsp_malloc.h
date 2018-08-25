/**
  *******************************************************************************************************
  * File Name: bsp_malloc.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-21
  * Brief: 内存管理模块
  *******************************************************************************************************
  * History
	*		1.Author: Vector
  *			Date: 2018-2-11
  *			Mod: 建立文件
  *******************************************************************************************************
  */
# ifndef __BSP_MALLOC_H
# define __BSP_MALLOC_H


# define USE_SRAMIN		1u   /* 是否使用内部SRAM */
# define USE_SRAMEX		0u   /* 是否使用外部SRAM */
# define USE_SRAMCCM	0u   /* 是否使用内部CCM内存，对F1无效 */

/* 内存块的总数 */
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
	# define SRAMIN_BLOCK_SIZE	32                  /*  单个内存块的大小为32个字节  */
	# define SRAMIN_MAX_SIZE		96*1024							/*  最大内存96  */	
	# define SRAMIN_TABLE_SIZE	SRAMIN_MAX_SIZE/SRAMIN_BLOCK_SIZE			/*  内存管理表的大小  */
# endif

# if USE_SRAMEX > 0u
	# define SRAMEX_BASE					0X68000000
	# define SRAMEX_BLOCK_SIZE		32						/*  最大内存块的大小为32字节  */
	# define SRAMEX_MAX_SIZE			960*1024			/*  最大管理960个字节内存  */	
	# define SRAMEX_TABLE_SIZE	SRAMEX_MAX_SIZE/SRAMEX_BLOCK_SIZE		/*  内存管理表的大小  */
# endif

# if USE_SRAMCCM > 0u
	# define SRAMCCM_BASE					0X10000000
	# define SRAMCCM_BLOCK_SIZE		32				/*  单个内存块的大小为32字节  */
	# define SRAMCCM_MAX_SIZE			60*1024		/*  最大管理60个字节  */
	# define SRAMCCM_TABLE_SIZE	SRAMCCM_MAX_SIZE/SRAMCCM_BLOCK_SIZE			/*  内存管理表的大小  */
# endif


/*    */
typedef struct
{
	void (*malloc_config)(void );   	/*  初始化函数  */
# if USE_SRAMEX > 0u 								/*  如果使用外部SRAM，则需要提供外部SRAM初始化函数  */
	void (*sram_config)(void);				/*  外部SRAM初始化函数  */
# endif
	uint8_t (*persued)(uint8_t );			/*  内存使用率  */
	uint8_t *membase[SRAMBANK];				/*  内存池基地址  */
	uint16_t *memmap[SRAMBANK];				/*  内存管理表  */
	uint8_t memready[SRAMBANK];				/*  内存池就绪指示  */
}MEM_TypeDef;

extern MEM_TypeDef malloc_dev;				/*    */

void bsp_mem_Set(void *s, uint8_t value, uint32_t count);		/*  设置内存  */
void bsp_mem_Copy(void *des, void *src, uint32_t count);			/*  复制内存  */
void bsp_mem_Config(void);																		/*  内存管理初始化  */
uint8_t bsp_mem_GetUsed(uint8_t memx);												/*  获取内存使用率  */

void bsp_mem_Free(uint8_t memx, void *mem);										/*  内存申请  */
void *bsp_mem_Alloc(uint8_t memx, uint32_t size);							/*  内存释放  */
void *bsp_mem_Realloc(uint8_t memx, void *mem, uint32_t size);	/*  重新分配内存  */
# endif

/********************************************  END OF FILE  *******************************************/


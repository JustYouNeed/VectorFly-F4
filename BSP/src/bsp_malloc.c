/**
  *******************************************************************************************************
  * File Name: bsp_malloc.c
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
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

# if USE_SRAMIN > 0u
/*  先申请一个大数组，用作内存池  */
__align(32) uint8_t MEM_SRAMIN[SRAMIN_MAX_SIZE];
/*  内存管理的地址表  */
uint16_t MAP_SRAMIN[SRAMIN_TABLE_SIZE];
# endif

# if USE_SRAMEX > 0u
/*  先在SRAM中申请一个大数组，用作内存池  */
__align(32) uint8_t MEM_SRAMEX[SRAMEX_MAX_SIZE] __attribute__((at(SRAMEX_BASE)));
/*  内存管理的地址表  */
uint16_t MAP_SRAMEX[SRAMEX_TABLE_SIZE] __attribute__((at(SRAMEX_BASE + SRAMEX_MAX_SIZE)));
# endif

# if USE_SRAMCCM > 0u
/*  在CCM内存中申请一个大数组，用作内存池  */
__align(32) uint8_t MEM_SRAMCCM[SRAMCCM_MAX_SIZE] __attribute__((at(SRAMCCM_BASE)));
/* 内存管理的地址表   */
uint16_t MAP_SRAMCCM[SRAMCCM_TABLE_SIZE] __attribute__((at(SRAMCCM_BASE + SRAMCCM_MAX_SIZE)));
# endif





/*  内存管理的地址表  */
const uint32_t MEM_TABLE_SIZE[SRAMBANK] = {
																						# if USE_SRAMIN > 0u
																						SRAMIN_TABLE_SIZE,
																						# endif

																						# if USE_SRAMEX > 0u
																						SRAMEX_TABLE_SIZE, 
																						# endif

																						# if USE_SRAMCCM > 0u	
																						SRAMCCM_TABLE_SIZE
																						# endif
																						};
/*  内存管理的地址表  */
const uint32_t MEM_BLOCK_SIZE[SRAMBANK] = {
																						# if USE_SRAMIN > 0u
																						SRAMIN_BLOCK_SIZE, 
																						# endif
																							
																						# if USE_SRAMEX > 0u
																						SRAMEX_BLOCK_SIZE, 
																						# endif
																							
																						# if USE_SRAMCCM > 0u	
																						SRAMCCM_BLOCK_SIZE
																						# endif
																							};
/*  内存管理的地址表  */
const uint32_t MEM_SIZE[SRAMBANK] = {
																			# if USE_SRAMIN > 0u
																			SRAMIN_MAX_SIZE,
																			# endif 
																				
																			# if USE_SRAMEX > 0u
																			SRAMEX_MAX_SIZE, 
																			# endif
																				
																			# if USE_SRAMCCM > 0u	
																			SRAMCCM_MAX_SIZE
																			# endif
																			};


MEM_TypeDef malloc_dev = 
{
	bsp_mem_Config,
	# if USE_SRAMEX > 0u
	FSMC_SRAM_Init,
	# endif
	bsp_mem_GetUsed,

	# if USE_SRAMIN > 0u
	MEM_SRAMIN,
	# endif
	
	# if USE_SRAMEX > 0u
	MEM_SRAMEX,
	# endif
	
	# if USE_SRAMCCM > 0u	
	MEM_SRAMCCM,
	# endif
	
	# if USE_SRAMIN > 0u
	MAP_SRAMIN,
	# endif
	
	# if USE_SRAMEX > 0u
	MAP_SRAMEX,
	# endif
	
	# if USE_SRAMCCM > 0u	
	MAP_SRAMCCM,
	# endif
	
	# if USE_SRAMIN > 0u
	0,
	# endif
	
	# if USE_SRAMEX > 0u
	0,
	# endif
	
	# if USE_SRAMCCM > 0u	
	0,
	# endif
};

/*
*********************************************************************************************************
*                        bsp_mem_Set                  
*
* Description: 设置一段内存的值
*             
* Arguments  : 1> s: 内存地址
*							 2> value: 要设置的值
*							 3> count: 内存长度
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
void bsp_mem_Set(void *s, uint8_t value, uint32_t count)
{
	uint8_t *xs = s;
	while(count -- ) *xs++ = value;
}

/*
*********************************************************************************************************
*                      bsp_mem_Copy                    
*
* Description: 将一段内存的值复制到另一段内存中
*             
* Arguments  : 1> des: 目标内存地址
*							 2> src: 源内存地址
*							 3> count: 要复制的长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_mem_Copy(void *des, void *src, uint32_t count)
{
	uint8_t *xdes = des;
	uint8_t *xsrc = src;
	while(count -- )*xdes++ = *xsrc++;
}

/*
*********************************************************************************************************
*                          bsp_mem_Config                
*
* Description: 初始化内存管理模块
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_mem_Config(void)
{
# if USE_SRAMIN > 0u		/* 如果使能了内部SRAM   */
	bsp_mem_Set(malloc_dev.memmap[SRAMIN], 0, MEM_TABLE_SIZE[SRAMIN] * 2);
	bsp_mem_Set(malloc_dev.membase[SRAMIN], 0, MEM_SIZE[SRAMIN]);
	malloc_dev.memready[SRAMIN] = 1;
# endif
	
# if USE_SRAMEX > 0u			/*  外部SRAM  */
	malloc_dev.sram_config();
	bsp_mem_Set(malloc_dev.memmap[SRAMEX], 0, MEM_TABLE_SIZE[SRAMEX] * 2);
	bsp_mem_Set(malloc_dev.membase[SRAMEX], 0, MEM_SIZE[SRAMEX]);
	malloc_dev.memready[SRAMEX] = 1;
# endif
# if USE_SRAMCCM > 0u			/*  CCM内存,只有F4系列有  */
	bsp_mem_Set(malloc_dev.memmap[SRAMCCM], 0, MEM_TABLE_SIZE[SRAMCCM] * 2);
	bsp_mem_Set(malloc_dev.membase[SRAMCCM], 0, MEM_SIZE[SRAMCCM]);
	malloc_dev.memready[SRAMCCM] = 1;
# endif
}


/*
*********************************************************************************************************
*                       bsp_mem_GetUsed                   
*
* Description: 获取内存使用情况
*             
* Arguments  : 1> memx: 内存块
*
* Reutrn     : 内存使用率
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t bsp_mem_GetUsed(uint8_t memx)
{
	uint32_t used = 0;
	uint32_t cnt = 0;
	
	/*  遍历整个内存,统计使用中的内存块  */
	for(cnt = 0; cnt < MEM_TABLE_SIZE[memx]; cnt++)
	{
		if(malloc_dev.memmap[memx][cnt]) used ++;
	}
	return (used * 100 )/(MEM_TABLE_SIZE[memx]);
}


/*
*********************************************************************************************************
*                            bsp_Malloc_IN              
*
* Description: 本函数内部申请内存用的函数
*             
* Arguments  : 1> memx: 内存块编号
*							 2> size: 要申请的内存的大小
*
* Reutrn     : 申请到的内存的地址
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint32_t bsp_Malloc_IN(uint8_t memx, uint32_t size)
{
	signed long offset = 0;
	uint32_t nmemblocks;
	uint32_t cmemblocks;
	uint32_t i;
	
	/*  如果内存没有初始化的话需要先初始化  */
	if(! malloc_dev.memready[memx])malloc_dev.malloc_config();
	
	/*  内存大小不符合要求  */
	if(size == 0) return 0XFFFFFFFF;
	nmemblocks = size/MEM_BLOCK_SIZE[memx];
	if(size % MEM_BLOCK_SIZE[memx])nmemblocks++;
	for(offset = MEM_TABLE_SIZE[memx] - 1; offset >= 0; offset --)
	{
		if(!malloc_dev.memmap[memx][offset]) cmemblocks ++;
		else cmemblocks = 0;
		if(cmemblocks == nmemblocks)
		{
			for(i = 0; i< nmemblocks; i++)
			{
				malloc_dev.memmap[memx][offset + i] = nmemblocks;
			}
			
			return (offset * MEM_BLOCK_SIZE[memx]);
		}
	}
	return 0XFFFFFFFF;
}


/*
*********************************************************************************************************
*                              bsp_Free_IN            
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
uint8_t bsp_Free_IN(uint8_t memx, uint32_t offset)
{
	int i = 0;
	int index = 0;
	int nmemblocks = 0;
	if(!malloc_dev.memready[memx]) 
	{
		malloc_dev.malloc_config();
		return 1;
	}
	if(offset < MEM_SIZE[memx])
	{
		index = offset / MEM_BLOCK_SIZE[memx];
		nmemblocks = malloc_dev.memmap[memx][index];
		for(i = 0; i < nmemblocks; i++)
		{
			malloc_dev.memmap[memx][index + i] = 0;
		}
		return 0;
	}else 
		return 2;
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
void *bsp_mem_Alloc(uint8_t memx, uint32_t size)
{
	uint32_t offset = 0;
	offset = bsp_Malloc_IN(memx, size);
	if(offset == 0XFFFFFFF) return NULL;
	else return (void *)((uint32_t )malloc_dev.membase[memx] + offset);
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
void bsp_mem_Free(uint8_t memx, void *mem)
{
	uint32_t offset = 0;
	if(mem == NULL) return;
	offset = (uint32_t)mem - (uint32_t)malloc_dev.membase[memx];
	bsp_Free_IN(memx, offset);
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
void *bsp_mem_Realloc(uint8_t memx, void *mem, uint32_t size)
{
	uint32_t offset = 0;
	offset = bsp_Malloc_IN(memx, size);
	if(0XFFFFFFFF == offset) return NULL;
	else
	{
		bsp_mem_Copy((void *)((uint32_t)malloc_dev.membase[memx] + offset), mem, size);
		bsp_mem_Free(memx, mem);
		return (void*)((uint32_t)malloc_dev.membase[memx]+offset);
	}
}

/********************************************  END OF FILE  *******************************************/




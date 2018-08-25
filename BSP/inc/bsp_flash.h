/**
  *******************************************************************************************************
  * File Name: bsp_flash.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-19
  * Brief: 外部SPI-Flash驱动模块
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-19
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
	
# ifndef __BSP_FLASH_H
# define __BSP_FLASH_H

uint8_t 	bsp_flash_Config(void);
uint16_t 	bsp_flash_ReadID(void);
uint8_t 	bsp_flash_ReadSR(void);
uint32_t 	bsp_flash_GetFlashSize(void);
void 			bsp_flash_WriteSR(uint8_t sr);
void 			bsp_flash_WriteCmd(FunctionalState state);
void 			bsp_flash_WritePage(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_WriteNoCheck(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_Read(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_Write(uint8_t *buff, uint32_t addr, uint16_t len);
void 			bsp_flash_EraseChip(void);
void 			bsp_flash_EraseSector(uint32_t dstAddr);
void 			bsp_flash_WaitBusy(void);
void 			bsp_flash_PowerCmd(FunctionalState state);



# endif

/********************************************  END OF FILE  *******************************************/

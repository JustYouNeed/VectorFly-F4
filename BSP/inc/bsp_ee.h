/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	

# ifndef __BSP_EE_H
# define __BSP_EE_H


# define AT24C01		127
# define AT24C02		255
# define AT24C04		511
# define AT24C08		1023
# define AT24C16		2047
# define AT24C32		4095
# define AT24C64	    8191
# define AT24C128	16383
# define AT24C256	32767  

# define EE_TYPE AT24C02


# define EE_SDA_PORT		GPIOD
# define EE_SDA_PIN			GPIO_Pin_2

# define EE_SCL_PORT		GPIOC
# define EE_SCL_PIN			GPIO_Pin_12

void EE_SDA_IN(void);
void EE_SDA_OUT(void);
void EE_SDA_LOW(void);
void EE_SDA_HIGH(void);
uint8_t EE_READ_SDA(void);

void EE_SCL_HIGH(void);
void EE_SCL_LOW(void);


void bsp_ee_Config(void);
uint8_t bsp_ee_Check(void);


uint8_t bsp_ee_ReadShortByte(uint16_t addr);
uint32_t bsp_ee_ReadLongByte(uint16_t addr, uint8_t bytelen);
void bsp_ee_WriteShortByte(uint16_t addr, uint8_t byte);
void bsp_ee_writeLongByte(uint16_t addr, uint32_t byte, uint8_t bytelen);

void bsp_ee_ReadBytes(uint16_t addr, uint8_t *bytes, uint16_t len);
void bsp_ee_WriteBytes(uint16_t addr, uint8_t *bytes, uint16_t len);

# endif

/********************************************  END OF FILE  *******************************************/


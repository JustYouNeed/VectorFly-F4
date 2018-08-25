/**
  *******************************************************************************************************
  * File Name: bsp_magnet.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-8-24
  * Brief: MPU9250内部磁力计驱动
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-8-24
	*			Mod: 建立文件,添加基本函数
  *
  *******************************************************************************************************
  */	
# ifndef __BSP_MAGNET_H
# define __BSP_MAGNET_H

#define MAG_ADDRESS_00         0x0C
#define MAG_ADDRESS_01         0x0D
#define MAG_ADDRESS_10         0x0E // default for InvenSense MPU-6050 evaluation board
#define MAG_ADDRESS_11         0x0F
#define MAG_DEFAULT_ADDRESS    MAG_ADDRESS_00

# define MAG_ADDR				0x18

#define MAG_RA_WIA             0x00
#define MAG_RA_INFO            0x01
#define MAG_RA_ST1             0x02
#define MAG_RA_HXL             0x03
#define MAG_RA_HXH             0x04
#define MAG_RA_HYL             0x05
#define MAG_RA_HYH             0x06
#define MAG_RA_HZL             0x07
#define MAG_RA_HZH             0x08
#define MAG_RA_ST2             0x09
#define MAG_RA_CNTL            0x0A
#define MAG_RA_RSV             0x0B // RESERVED, DO NOT USE
#define MAG_RA_ASTC            0x0C
#define MAG_RA_TS1             0x0D // SHIPMENT TEST, DO NOT USE
#define MAG_RA_TS2             0x0E // SHIPMENT TEST, DO NOT USE
#define MAG_RA_I2CDIS          0x0F
#define MAG_RA_ASAX            0x10
#define MAG_RA_ASAY            0x11
#define MAG_RA_ASAZ            0x12

#define MAG_ST1_DRDY_BIT       0

#define MAG_ST2_HOFL_BIT       3
#define MAG_ST2_DERR_BIT       2

#define MAG_CNTL_MODE_BIT      3
#define MAG_CNTL_MODE_LENGTH   4

#define MAG_MODE_POWERDOWN     0x00
#define MAG_MODE_SINGLE        0x01
#define MAG_MODE_CONT1         0x02
#define MAG_MODE_CONT2         0x06
#define MAG_MODE_EXTTRIG       0x04
#define MAG_MODE_SELFTEST      0x08
#define MAG_MODE_FUSEROM       0x0F
#define MAG_MODE_14BIT         0x00
#define MAG_MODE_16BIT         0x10

#define MAG_ASTC_SELF_BIT      6

#define MAG_I2CDIS			0x1B
#define MAG_I2CDIS_BIT         0

#define MAG_ST_X_MIN           (int16_t)(-200)
#define MAG_ST_X_MAX           (int16_t)(200)
#define MAG_ST_Y_MIN           (int16_t)(-200)
#define MAG_ST_Y_MAX           (int16_t)(200)
#define MAG_ST_Z_MIN           (int16_t)(-3200)
#define MAG_ST_Z_MAX           (int16_t)(-800)

#define MAG_GAUSS_PER_LSB		(float)(666.7f)

void bsp_mag_Init(void);
bool bsp_mag_Test(void);

uint8_t bsp_mag_GetDeviceID(void);
uint8_t bsp_mag_GetInfo(void);
uint8_t bsp_mag_GetDataReady(void);

// H* registers
void bsp_mag_GetHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t bsp_mag_GetHeadingX(void);
int16_t bsp_mag_GetHeadingY(void);
int16_t bsp_mag_GetHeadingZ(void);

bool bsp_mag_GetOverflowStatus(void);// ST2 寄存器
bool bsp_mag_GetDataError(void);

uint8_t bsp_mag_GetMode(void);// CNTL 寄存器
void bsp_mag_SetMode(uint8_t mode);
void bsp_mag_Reset(void);

void bsp_mag_SetSelfTest(bool enabled);// ASTC 寄存器

void bsp_mag_DisableI2C(void); // I2CDIS

// ASA* registers
void bsp_mag_GetAdjustment(int8_t *x, int8_t *y, int8_t *z);
void bsp_mag_SetAdjustment(int8_t x, int8_t y, int8_t z);
uint8_t bsp_mag_GetAdjustmentX(void);
void bsp_mag_SetAdjustmentX(u8 x);
uint8_t bsp_mag_GetAdjustmentY(void);
void bsp_mag_SetAdjustmentY(u8 y);
uint8_t bsp_mag_GetAdjustmentZ(void);
void bsp_mag_SetAdjustmentZ(u8 z);
# endif

/********************************************  END OF FILE  *******************************************/



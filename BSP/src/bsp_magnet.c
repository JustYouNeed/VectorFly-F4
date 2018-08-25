/**
  *******************************************************************************************************
  * File Name: bsp_mag_net.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-8-24
  * Brief: MPU9250ÄÚ²¿´ÅÁ¦¼ÆÇý¶¯
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-8-24
	*			Mod: ½¨Á¢ÎÄ¼þ,Ìí¼Ó»ù±¾º¯Êý
  *
  *******************************************************************************************************
  */	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"


# define bsp_mag_DelayUs		bsp_tim_DelayUs
# define bsp_mag_DelayMs		bsp_tim_DelayMs

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static i2cDevice mag_i2c;
static uint8_t buffer[8];

extern i2cDevice mpu_i2c;
void bsp_mag_Init(void)
{
	mag_i2c = mpu_i2c;
}


static bool bsp_mag_EvaluateSelfTest(int16_t min, int16_t max, int16_t value, char* string)
{
	if (value < min || value > max)
	{
		printf("Self test %s [FAIL]. low: %d, high: %d, measured: %d\n", string, min, max, value);
		return false;
	}
	return true;
}


/*MAG×Ô¼ì*/
bool bsp_mag_SelfTest(void)
{
	bool testStatus = true;
	int16_t mx, my, mz;  // positive magnetometer measurements
	uint8_t confSave;
	uint8_t timeout = 20;
	
	if (i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_CNTL, &confSave) == false)
	{
		// TODO: error handling
		return false;
	}

	bsp_mag_SetMode(MAG_MODE_POWERDOWN);
	bsp_mag_SetSelfTest(true);
	bsp_mag_SetMode(MAG_MODE_16BIT | MAG_MODE_SELFTEST);
	bsp_mag_GetOverflowStatus();// Clear ST1 by reading ST2
	
	while (!bsp_mag_GetDataReady() && timeout--)
	{
		bsp_mag_DelayMs(1);
	}
	bsp_mag_GetHeading(&mx, &my, &mz);
	bsp_mag_SetMode(MAG_MODE_POWERDOWN);

	if (bsp_mag_EvaluateSelfTest(MAG_ST_X_MIN, MAG_ST_X_MAX, mx, "X") &&
		bsp_mag_EvaluateSelfTest(MAG_ST_Y_MIN, MAG_ST_Y_MAX, my, "Y") &&
		bsp_mag_EvaluateSelfTest(MAG_ST_Z_MIN, MAG_ST_Z_MAX, mz, "Z"))
	{
		printf("MAG Self test [OK].\n");
	}else
	{
		printf("MAG Self test [FAIL].\n");
		testStatus = false;
	}
	
	bsp_mag_SetMode(confSave);// Power up with saved config

	return testStatus;
}

uint8_t bsp_mag_GetDeviceID()
{
	i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_WIA, buffer);
	return buffer[0];
}

// INFO ¼Ä´æÆ÷
uint8_t bsp_mag_GetInfo()
{
	i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_INFO, buffer);
	return buffer[0];
}

// ST1 ¼Ä´æÆ÷
uint8_t bsp_mag_GetDataReady()
{
	i2c_ReadBit(mag_i2c, MAG_ADDR, MAG_RA_ST1, MAG_ST1_DRDY_BIT, buffer);
	return buffer[0];
}

// H* ¼Ä´æÆ÷
void bsp_mag_GetHeading(int16_t *x, int16_t *y, int16_t *z)
{
	i2c_ReadBuff(mag_i2c, MAG_ADDR, MAG_RA_HXL, 6, buffer);
	*x = (((int16_t) buffer[1]) << 8) | buffer[0];
	*y = (((int16_t) buffer[3]) << 8) | buffer[2];
	*z = (((int16_t) buffer[5]) << 8) | buffer[4];
	
//	bsp_mag_SetMode(MAG_MODE_CONT2);
}
int16_t bsp_mag_GetHeadingX()
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_CNTL, MAG_MODE_SINGLE);
	i2c_ReadBuff(mag_i2c, MAG_ADDR, MAG_RA_HXL, 2, buffer);
	return (((int16_t) buffer[1]) << 8) | buffer[0];
}
int16_t bsp_mag_GetHeadingY()
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_CNTL, MAG_MODE_SINGLE);
	i2c_ReadBuff(mag_i2c, MAG_ADDR, MAG_RA_HYL, 2, buffer);
	return (((int16_t) buffer[1]) << 8) | buffer[0];
}
int16_t bsp_mag_GetHeadingZ()
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_CNTL, MAG_MODE_SINGLE);
	i2c_ReadBuff(mag_i2c, MAG_ADDR, MAG_RA_HZL, 2, buffer);
	return (((int16_t) buffer[1]) << 8) | buffer[0];
}

// ST2 ¼Ä´æÆ÷
bool bsp_mag_GetOverflowStatus()
{
	i2c_ReadBit(mag_i2c, MAG_ADDR, MAG_RA_ST2, MAG_ST2_HOFL_BIT, buffer);
	return buffer[0];
}
bool bsp_mag_GetDataError()
{
	i2c_ReadBit(mag_i2c, MAG_ADDR, MAG_RA_ST2, MAG_ST2_DERR_BIT, buffer);
	return buffer[0];
}

// CNTL ¼Ä´æÆ÷
uint8_t bsp_mag_GetMode()
{
	i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_CNTL, buffer);
	return buffer[0];
}
void bsp_mag_SetMode(uint8_t mode)
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_CNTL, mode);
}
void bsp_mag_Reset()
{
	i2c_WriteBits(mag_i2c, MAG_ADDR, MAG_RA_CNTL, MAG_CNTL_MODE_BIT,
					MAG_CNTL_MODE_LENGTH, MAG_MODE_POWERDOWN);
}

// ASTC ¼Ä´æÆ÷
void bsp_mag_SetSelfTest(bool enabled)
{
	i2c_WriteBit(mag_i2c, MAG_ADDR, MAG_RA_ASTC, MAG_ASTC_SELF_BIT, enabled);
}

// I2CDIS
void bsp_mag_DisableI2C()
{
	i2c_WriteBit(mag_i2c, MAG_ADDR, MAG_RA_I2CDIS, MAG_I2CDIS_BIT, true);
}

// ASA* ¼Ä´æÆ÷
void bsp_mag_GetAdjustment(int8_t *x, int8_t *y, int8_t *z)
{
	i2c_ReadBuff(mag_i2c, MAG_ADDR, MAG_RA_ASAX, 3, buffer);
	*x = buffer[0];
	*y = buffer[1];
	*z = buffer[2];
}
void bsp_mag_SetAdjustment(int8_t x, int8_t y, int8_t z)
{
	buffer[0] = x;
	buffer[1] = y;
	buffer[2] = z;
	i2c_WriteBuff(mag_i2c, MAG_ADDR, MAG_RA_ASAX, 3, buffer);
}
uint8_t bsp_mag_GetAdjustmentX()
{
	i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_ASAX, buffer);
	return buffer[0];
}
void bsp_mag_SetAdjustmentX(uint8_t x)
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_ASAX, x);
}
uint8_t bsp_mag_GetAdjustmentY()
{
	i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_ASAY, buffer);
	return buffer[0];
}
void bsp_mag_SetAdjustmentY(uint8_t y)
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_ASAY, y);
}
uint8_t bsp_mag_GetAdjustmentZ()
{
	i2c_ReadByte(mag_i2c, MAG_ADDR, MAG_RA_ASAZ, buffer);
	return buffer[0];
}
void bsp_mag_SetAdjustmentZ(uint8_t z)
{
	i2c_WriteByte(mag_i2c, MAG_ADDR, MAG_RA_ASAZ, z);
}


/********************************************  END OF FILE  *******************************************/


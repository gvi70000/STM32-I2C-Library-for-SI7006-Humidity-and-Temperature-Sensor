#include "SI7006.h"

#define SI7006_I2C_ADDRESS		(0x40 << 1)
#define SI7006_Humi_HM        0xE5 // Measure Relative Humidity, Hold Master Mode
#define SI7006_Humi_NHM       0xF5 // Measure Relative Humidity, No Hold Master Mode
#define SI7006_Temp_HM        0xE3 // Measure Temperature, Hold Master Mode
#define SI7006_Temp_NHM       0xF3 // Measure Temperature, No Hold Master Mode
#define SI7006_Temp_AH        0xE0 // Read Temperature Value from Previous RH Measurement
#define SI7006_Reset					0xFE // Reset
#define SI7006_W_RHT_U_reg    0xE6 // Write RH/T User Register 1
#define SI7006_R_RHT_U_reg    0xE7 // Read RH/T User Register 1
#define SI7006_W_Heater_C_reg 0x51 // Write Heater Control Register
#define SI7006_R_Heater_C_reg 0x11 // Read Heater Control Register
#define SI7006_R_ID_Byte11    0xFA // Read Electronic ID 1st Byte, first part
#define SI7006_R_ID_Byte12    0x0F // Read Electronic ID 1st Byte, second part
#define SI7006_R_ID_Byte21    0xFC // Read Electronic ID 2nd Byte, first part
#define SI7006_R_ID_Byte22    0xC9 // Read Electronic ID 2nd Byte, second part
#define SI7006_R_Firm_rev1    0x84 // Read Firmware Revision, first part
#define SI7006_R_Firm_rev2    0xB8 // Read Firmware Revision, second part
#define SI7006_R_Firm					((SI7006_R_Firm_rev2<<8) | SI7006_R_Firm_rev1) // Read Firmware Revision, second part

#define SI7006_H12_T14				0x00
#define SI7006_H8_T12					0x01
#define SI7006_H10_T13				0x80
#define SI7006_H11_T11				0x81
#define SI7006_RES0						0
#define SI7006_RES1						7
#define SI7006_VDDS						6
#define SI7006_HTRE						2

#define SI7006_HEATER_OFF			3 // current value in mA for register value 0
#define SI7006_HEATER_STEP		6 // mA/LSB

#define SI7006_RESPONCE_TIME   500

uint8_t SI7006_USER_REG = 0x3A;
uint8_t SI7006_HEATER_REG = 0x00;

static float process_temp_code(uint16_t temp_code)
{
  return (float)(((175.72f * temp_code) / 65536.0f) - 46.85f);
}

static float process_humi_code(uint16_t humi_code)
{
  float value = (float)(((125.0f * humi_code) / 65536.0f) - 6.0f);

  if(value < 0.0f)
    return 0.0f;
  else if(value > 100.0f)
    return 100.0f;
  else
    return value;
}

static int8_t r_reg(uint8_t reg)
{
  uint8_t cmd;
  uint8_t* data;

  if(reg == SI7006_USER_REG)
    cmd = SI7006_R_RHT_U_reg;
  else 
    cmd = SI7006_R_Heater_C_reg;
	data = &(reg);

  if(HAL_OK != HAL_I2C_Mem_Read(&hi2c1, SI7006_I2C_ADDRESS, cmd, 1, data, 1, SI7006_RESPONCE_TIME))
    return -1;
  else
    return 0;
}

static int8_t w_reg(uint8_t value, uint8_t reg) {
  uint16_t cmd;
  if(reg == SI7006_USER_REG)
    cmd = SI7006_W_RHT_U_reg;
  else 
    cmd = SI7006_W_Heater_C_reg;

  if(HAL_OK != HAL_I2C_Mem_Write(&hi2c1, SI7006_I2C_ADDRESS, cmd, 1, &value, 1, SI7006_RESPONCE_TIME))
    return -1;
  else
    return 0;
}

void SI7006_Init(void) {
	SI7006_setResolution(SI7006_H12_T14);
	//SI7006_EnableHeater(1);
}

uint8_t SI7006_getFirmwareRevision(void) {
  uint8_t data;
	uint16_t cmd = SI7006_R_Firm;
  if(HAL_OK != HAL_I2C_Master_Transmit(&hi2c1, SI7006_I2C_ADDRESS, (uint8_t*)(&cmd), 2, SI7006_RESPONCE_TIME))
    return 0;
  if(HAL_OK != HAL_I2C_Master_Receive(&hi2c1, SI7006_I2C_ADDRESS, &data, 1, SI7006_RESPONCE_TIME))
    return 0;
	else
		return data;
}

void SI7006_Read(float* humidity, float* temperature) {
  uint8_t cmd = SI7006_Humi_HM;
  uint8_t buffer[2];
  uint16_t code;

  if(HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, SI7006_I2C_ADDRESS, &cmd, 1, SI7006_RESPONCE_TIME)){
		if(HAL_OK == HAL_I2C_Master_Receive(&hi2c1, SI7006_I2C_ADDRESS, buffer, 2, SI7006_RESPONCE_TIME)){
			code = (uint16_t)((buffer[0]<<8) | buffer[1]);
			*humidity = process_humi_code(code);
		} else
			*humidity = 0.0f;
	} else
		*humidity = 0.0f;
	
  /* There is a temperature measurement with each RH measurement */
  cmd = SI7006_Temp_AH;

  if(HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, SI7006_I2C_ADDRESS, &cmd, 1, SI7006_RESPONCE_TIME)){
		if(HAL_OK == HAL_I2C_Master_Receive(&hi2c1, SI7006_I2C_ADDRESS, buffer, 2, SI7006_RESPONCE_TIME)){
			code = (uint16_t)((buffer[0]<<8) | buffer[1]);
			*temperature = process_temp_code(code);
		} else
			*temperature = 0.0f;				
	} else
		*temperature = 0.0f;
}

void SI7006_setResolution(uint8_t resolution) {
  int8_t rv;
  uint8_t temp = SI7006_USER_REG;

  switch(resolution)  {
    case SI7006_H8_T12:
      SI7006_USER_REG &= (uint8_t)(~(1<<SI7006_RES1));
      SI7006_USER_REG |= (1<<SI7006_RES0);
			//0x3B
      break;
    case SI7006_H10_T13:
      SI7006_USER_REG &= ~(1<<SI7006_RES0);
      SI7006_USER_REG |= (1<<SI7006_RES1);
			//0xBA
      break;
    case SI7006_H11_T11:
      SI7006_USER_REG |= (1<<SI7006_RES1) | (1<<SI7006_RES0);
			//0xBB
      break;
    default:
      SI7006_USER_REG &= (uint8_t)(~(1<<SI7006_RES1) & ~(1<<SI7006_RES0));
			//0x3A
  }
	rv = w_reg(SI7006_USER_REG, SI7006_USER_REG);
  /* in case of write error restore local copy of the register value */
  if(rv < 0)
    SI7006_USER_REG = temp;
}

uint8_t SI7006_getResolution(void) {
  return (SI7006_USER_REG & ((1<<SI7006_RES1) | (1<<SI7006_RES0)));
}

void SI7006_setHeaterCurrent(uint8_t current) {
  uint8_t reg_val = (current - SI7006_HEATER_OFF)/SI7006_HEATER_STEP;
  if(reg_val > 0x0F)
    reg_val = 0x0F;
  w_reg(reg_val, SI7006_HEATER_REG);
  /* in case of write success update local copy of the register value */
  SI7006_HEATER_REG = reg_val;
}

int8_t SI7006_getHeaterCurrent(void) {
	r_reg(SI7006_HEATER_REG);
  return ((SI7006_HEATER_REG & (0x0F)) * SI7006_HEATER_STEP) + SI7006_HEATER_OFF;
}

int8_t SI7006_Warning(void) {
  if(r_reg(SI7006_USER_REG) < 0)
    return -1;

  if(SI7006_USER_REG & (1<<SI7006_VDDS))
    return 1;
  else
    return 0;
}

void SI7006_EnableHeater(uint8_t val) {
  int8_t rv;
  uint8_t temp = SI7006_USER_REG;

  if(val == 0)
    SI7006_USER_REG &= ~(1<<SI7006_HTRE);
  else
    SI7006_USER_REG |= (1<<SI7006_HTRE);

	rv = w_reg(SI7006_USER_REG, SI7006_USER_REG);
  /* in case of write error restore local copy of the register value */
  if(rv < 0)
    SI7006_USER_REG = temp;
}

void SI7006_RST(void) {
  uint8_t cmd = SI7006_Reset;
  HAL_I2C_Master_Transmit(&hi2c1, SI7006_I2C_ADDRESS, &cmd, 1, SI7006_RESPONCE_TIME);	
}




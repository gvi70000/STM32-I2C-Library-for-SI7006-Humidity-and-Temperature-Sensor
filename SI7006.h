#ifndef __SI7006_H__
#define __SI7006_H__

#include "stm32f1xx_hal.h"
#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>

void SI7006_Init(void);
uint8_t SI7006_getFirmwareRevision(void);
void SI7006_Read(float* humidity, float* temperature);
void SI7006_setResolution(uint8_t resolution);
uint8_t SI7006_getResolution(void);
void SI7006_setHeaterCurrent(uint8_t current);
int8_t SI7006_getHeaterCurrent(void);
int8_t SI7006_Warning(void);
void SI7006_EnableHeater(uint8_t val);
void SI7006_RST(void);

#endif  // __SI7006_H__
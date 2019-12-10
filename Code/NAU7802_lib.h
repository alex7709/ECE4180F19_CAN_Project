
#include "stm32f0xx_hal.h"

enum NAUchan
	{
	NAUCHANNEL_1 = 0, 
	NAUCHANNEL_2 = 1, 
	NAUSWITCH = 2
	};


extern uint8_t addr_NAU7802;
void NAU7802SwCh(I2C_HandleTypeDef, enum NAUchan channel);
void NAU7802Retrieve(I2C_HandleTypeDef i2c, volatile uint8_t tempArr[8]);
float NAU7802Retrievef(I2C_HandleTypeDef i2c);
void NAU7802_Init(I2C_HandleTypeDef i2c);
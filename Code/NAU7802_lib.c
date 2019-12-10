#include "NAU7802_lib.h"
uint8_t addr_NAU7802 = 0x2A;


float NAU7802Retrievef(I2C_HandleTypeDef i2c){
	int result; int result2; float resultf; uint8_t temp; uint8_t tempArr[8];
		NAU7802Retrieve(i2c, tempArr);
		result = tempArr[0];
		result = result << 8;
		result |= tempArr[1];
		result = result <<8;
		result |= tempArr[2];
		result2 = result <<8;
		result2 = result2>>8;
		return resultf = ((float)result2/(float)0x007FFFFF) *1.65f;
}
void NAU7802Retrieve(I2C_HandleTypeDef i2c, volatile uint8_t tempArr[8]){
	int result; int result2; float resultf; uint8_t temp;
		temp = 0x12;
		HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
		HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), tempArr, 3, 1E6);
}
void NAU7802SwCh(I2C_HandleTypeDef i2c, enum NAUchan channel){
	int8_t temp; uint8_t tempArr[8];
	if(channel == NAUSWITCH){
		temp = 0x02;
		HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
		HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6);
		tempArr[0] = 0x02;
		tempArr[1] = temp ^ 0x80;
		HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
		return;
	}
	else if(channel == NAUCHANNEL_1){
		temp = 0x02;
		HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
		HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6);
		tempArr[0] = 0x02;
		tempArr[1] = temp & ~0x80;
		HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
		return;
	}
	//assume now channel is 2
	temp = 0x02;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
	HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6);
	tempArr[0] = 0x02;
	tempArr[1] = temp | 0x80;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
	
}



void NAU7802_Init(I2C_HandleTypeDef i2c){
	int8_t temp; uint8_t tempArr[8];
	tempArr[0] = 0x00;
	tempArr[1] = 0x01; //RR bit high
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
	tempArr[0] = 0x00;
	tempArr[1] = 0x02; //RR bit low and PUD high
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
	do{ //checking pwr read bit
		temp = 0x00;
		HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
		HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6); //if this doesn't work then probably need to or the address with read
	}while(!(temp & 0x08));
	tempArr[0] = 0x00;
	tempArr[1] = 0xAE;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
	tempArr[0] = 0x15;
	tempArr[1]= 0x30;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
	temp = 0x1B;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
	HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6);
	tempArr[0] = 0x1B;
	tempArr[1] = temp | 0x10;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
	
	temp = 0x02;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6 );
	HAL_I2C_Master_Receive(&i2c, (addr_NAU7802<<1), &temp, 1, 1E6);
	tempArr[0] = 0x02;
	tempArr[1] = temp | 0xF0;
	HAL_I2C_Master_Transmit(&i2c, (addr_NAU7802<<1), tempArr, 2, 1E6 );
}
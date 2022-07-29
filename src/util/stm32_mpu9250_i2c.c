#include "stm32_mpu9250_i2c.h"

extern I2C_HandleTypeDef hi2c2;

int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	uint8_t data_for_transmit[length + 1];
	data_for_transmit[0] = reg_addr;

	for (unsigned char i = 0; i < length; i++)
	{
		data_for_transmit[i + 1] = data[i];
	}

	while (HAL_I2C_Master_Transmit(&hi2c2, slave_addr << 1, data_for_transmit, (length + 1), 0xff) != HAL_OK);

	return 0;
}

int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	while (HAL_I2C_Master_Transmit(&hi2c2, slave_addr << 1, &reg_addr, 1, 0xff) != HAL_OK);
	while (HAL_I2C_Master_Receive(&hi2c2, slave_addr << 1, data, length, 0xff) != HAL_OK);
	
	return 0;
}

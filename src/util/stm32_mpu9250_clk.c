#include "stm32_mpu9250_clk.h"

int stm32_get_clock_ms(unsigned long *count)
{
	*count = HAL_GetTick();
	return 0;
}

int stm32_delay_ms(unsigned long num_ms)
{
	HAL_Delay(num_ms);
	return 0;
}
#include "stm32_mpu9250_i2c.h"

#define I2C_SCL_Pin MPU_SCL_Pin
#define I2C_SCL_GPIO_Port MPU_SCL_GPIO_Port
#define I2C_SDA_Pin MPU_SDA_Pin
#define I2C_SDA_GPIO_Port MPU_SDA_GPIO_Port

extern I2C_HandleTypeDef hi2c2;

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
{
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;
    /* Wait until flag is set */
    for (; (state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);)
    {
        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
            {
                ret = 0;
            }
            else
            {
            }
        }
        asm("nop");
    }
    return ret;
}

static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef* handle, uint32_t timeout)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. Clear PE bit.
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(handle);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = I2C_SCL_Pin;
    HAL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C_SDA_Pin;
    HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;

    GPIO_InitStructure.Pin = I2C_SCL_Pin;
    HAL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C_SDA_Pin;
    HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handle->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(handle);
}

int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
    uint8_t data_for_transmit[length + 1];

    data_for_transmit[0] = reg_addr;

    for (unsigned char i = 0; i < length; i++)
    {
        data_for_transmit[i + 1] = data[i];
    }

    HAL_StatusTypeDef ret_V = HAL_I2C_Master_Transmit(&hi2c2, slave_addr << 1, data_for_transmit, (length + 1), 0xff);

    if (ret_V != HAL_OK)
    {
        I2C_ClearBusyFlagErratum(&hi2c2, 10);
    }

    return ret_V;
}

int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
    HAL_StatusTypeDef ret_V = HAL_I2C_Master_Transmit(&hi2c2, slave_addr << 1, &reg_addr, 1, 0xff);
    if (ret_V != HAL_OK)
    {
        I2C_ClearBusyFlagErratum(&hi2c2, 10);
        return ret_V;
    }

    ret_V = HAL_I2C_Master_Receive(&hi2c2, slave_addr << 1, data, length, 0xff);

    if (ret_V != HAL_OK)
    {
        I2C_ClearBusyFlagErratum(&hi2c2, 10);
    }

    return ret_V;
}

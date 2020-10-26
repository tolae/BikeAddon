#include "l3gd20.h"

void l3gd20_init(SPI_HandleTypeDef* spix)
{
	uint8_t whoami = 0;
	l3gd20_read(spix, &whoami, L3GD20_WHO_AM_I_ADDR, 1);
}

void l3gd20_read(SPI_HandleTypeDef *spix, uint8_t *out_buffer, uint8_t addr, uint8_t size)
{
	/* Set RW bit to 1 for reading */
	addr |= 1U << 7;
	/* If reading more than one byte, set MS pin */
	if (size > 1)
	{
		addr |= 1U << 6;
	}
	/* Set select line to low */
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spix, &addr, size, 0xFFFF);
	HAL_SPI_Receive(spix, out_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_SET);
}

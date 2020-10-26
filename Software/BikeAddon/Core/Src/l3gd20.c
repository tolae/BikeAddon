#include "l3gd20.h"

typedef struct
{
	union
	{
		uint8_t addr;
		struct
		{
			uint8_t _addr : 6;
			uint8_t ms : 1;
			uint8_t rw : 1;
		};
	};
} l3gd20_spiaddr_t;

typedef enum
{
	L3GD20_SPI_WRITE = 0,
	L3GD20_SPI_READ = 1
} L3GD20_RW;

void l3gd20_prep_addr(l3gd20_spiaddr_t* inout_addr, uint32_t rw, uint8_t size);

void l3gd20_init(SPI_HandleTypeDef* spix)
{
	/* Configure for 3-wire mode to use in tandem with LSM303C module */
	uint8_t ctrl_reg4_sim_set = 0x01;
	l3gd20_write(spix, &ctrl_reg4_sim_set, L3GD20_CTRL_REG4_ADDR, 1);
	uint8_t whoami = 0;
	l3gd20_read(spix, &whoami, L3GD20_WHO_AM_I_ADDR, 1);
}

void l3gd20_write(SPI_HandleTypeDef* spix, uint8_t* in_buffer, uint8_t addr, uint8_t size)
{
	l3gd20_prep_addr((l3gd20_spiaddr_t*)(&addr), L3GD20_SPI_WRITE, size);

	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spix, &addr, size, 0xFFFF);
	HAL_SPI_Transmit(spix, in_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_SET);
}

void l3gd20_read(SPI_HandleTypeDef *spix, uint8_t *out_buffer, uint8_t addr, uint8_t size)
{
	l3gd20_prep_addr((l3gd20_spiaddr_t*)(&addr), L3GD20_SPI_READ, size);

	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spix, &addr, size, 0xFFFF);
	HAL_SPI_Receive(spix, out_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_SET);
}

void l3gd20_prep_addr(l3gd20_spiaddr_t* inout_addr, uint32_t rw, uint8_t size)
{
	inout_addr->rw = rw;
	inout_addr->ms = size > 1 ? 1 : 0;
}

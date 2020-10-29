#include "l3gd20.h"

#define G_GAIN_X
#define G_GAIN_Y
#define G_GAIN_Z

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

SPI_HandleTypeDef* l3gd20_spi;

void l3gd20_init(SPI_HandleTypeDef* spix)
{
	l3gd20_spi = spix;

	/* Configure for 3-wire mode to use in tandem with LSM303C module */
	uint8_t config = 0x01;
	l3gd20_write(&config, L3GD20_CTRL_REG4_ADDR, 1);
	config = 0x3F;
	l3gd20_write(&config, L3GD20_CTRL_REG1_ADDR, 1);
	config = 0x20;
	l3gd20_write(&config, L3GD20_CTRL_REG2_ADDR, 1);
	config = 0x10;
	l3gd20_write(&config, L3GD20_CTRL_REG5_ADDR, 1);
	uint8_t whoami = 0;
	l3gd20_read(&whoami, L3GD20_WHO_AM_I_ADDR, 1);
}

void l3gd20_write( uint8_t* in_buffer, uint8_t addr, uint8_t size)
{
	l3gd20_prep_addr((l3gd20_spiaddr_t*)(&addr), L3GD20_SPI_WRITE, size);

	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(l3gd20_spi, &addr, size, 0xFFFF);
	HAL_SPI_Transmit(l3gd20_spi, in_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_SET);
}

void l3gd20_read(uint8_t *out_buffer, uint8_t addr, uint8_t size)
{
	l3gd20_prep_addr((l3gd20_spiaddr_t*)(&addr), L3GD20_SPI_READ, size);

	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(l3gd20_spi, &addr, size, 0xFFFF);
	HAL_SPI_Receive(l3gd20_spi, out_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(L3GD20_CS_GPIO_Port, L3GD20_CS_Pin, GPIO_PIN_SET);
}

void l3gd20_prep_addr(l3gd20_spiaddr_t* inout_addr, uint32_t rw, uint8_t size)
{
	inout_addr->rw = rw;
	inout_addr->ms = size > 1 ? 1 : 0;
}

IMPLEMENT_READ_XYZ_VALUES(l3gd20)
{
	/* Intermediately consider everything as signed 16 bit integer */
	int16_t x;
	int16_t y;
	int16_t z;

	l3gd20_read((uint8_t*)(&x), L3GD20_OUT_X_L_ADDR, 2);
	l3gd20_read((uint8_t*)(&y), L3GD20_OUT_Y_L_ADDR, 2);
	l3gd20_read((uint8_t*)(&z), L3GD20_OUT_Z_L_ADDR, 2);

	x = x * L3GD20_250DPS_SENSITIVITY;
	y = y * L3GD20_250DPS_SENSITIVITY;
	z = z * L3GD20_250DPS_SENSITIVITY;

	inout_xyz_axis->x = x;
	inout_xyz_axis->y = y;
	inout_xyz_axis->z = z;
}


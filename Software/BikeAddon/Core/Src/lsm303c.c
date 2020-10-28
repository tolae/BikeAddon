#include "lsm303c.h"

typedef struct
{
	union
	{
		uint8_t addr;
		struct
		{
			uint8_t _addr : 7;
			uint8_t rw : 1;
		};
	};
} lsm303c_spiaddr_t;

typedef enum
{
	LSM303C_SPI_WRITE = 0,
	LSM303C_SPI_READ = 1
} LSM303C_RW;

void lsm303c_prep_addr(lsm303c_spiaddr_t* inout_addr, uint32_t rw);

void lsm303c_init(SPI_HandleTypeDef* spix)
{
	uint8_t config;
	uint8_t whoami;
	/* Enable R/W mode for accelerometer and magnetometer */
	/* Have to read a dummy register first */
	HAL_Delay(100);
	lsm303c_read(spix, &config, LSM303C_CTRL_REG4_A_ADDR, 1);
	config = 0x01;
	lsm303c_write(spix, &config, LSM303C_CTRL_REG4_A_ADDR, 1);
	lsm303c_read(spix, &config, LSM303C_CTRL_REG4_A_ADDR, 1);

	do
	{
		HAL_Delay(100);
		lsm303c_read(spix, &whoami, LSM303C_WHO_AM_I_A_ADRR, 1);
	} while (whoami != LSM303C_WHO_AM_I_A_DEFAULT);
}

void lsm303c_write(SPI_HandleTypeDef* spix, uint8_t* in_buffer, uint8_t addr, uint8_t size)
{
	lsm303c_prep_addr((lsm303c_spiaddr_t*)(&addr), LSM303C_SPI_WRITE);

	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spix, &addr, size, 0xFFFF);
	HAL_SPI_Transmit(spix, in_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_SET);
}

void lsm303c_read(SPI_HandleTypeDef* spix, uint8_t* out_buffer, uint8_t addr, uint8_t size)
{
	lsm303c_prep_addr((lsm303c_spiaddr_t*)(&addr), LSM303C_SPI_READ);

	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spix, &addr, size, 0xFFFF);
	HAL_SPI_Receive(spix, out_buffer, size, 0xFFFF);
	HAL_GPIO_WritePin(LSM303C_XL_CS_GPIO_Port, LSM303C_XL_CS_Pin, GPIO_PIN_SET);
}

void lsm303c_prep_addr(lsm303c_spiaddr_t* inout_addr, uint32_t rw)
{
	inout_addr->rw = rw;
}

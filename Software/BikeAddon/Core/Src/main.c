/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "common.h"
#include "hall_effect.h"
#include "l3gd20.h"
#include "lsm303c.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include "alpha_beta_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MY_DEBUG 1
#define OUTPUT_AS_RAW 0
/* Front Wheel Timer Configurations */
#define FRONT_WHEEL_TIM htim2
#define FRONT_WHEEL_TIMX 2
#define FRONT_WHEEL_CHL TIM_CHANNEL_3
/* Back Wheel Timer Configurations */
#define BACK_WHEEL_TIM htim2
#define BACK_WHEEL_TIMX 2
#define BACK_WHEEL_CHL TIM_CHANNEL_4
/* Pedal Gear Timer Configurations */
#define PEDAL_GEAR_TIM htim1
#define PEDAL_GEAR_TIMX 1
#define PEDAL_GEAR_CHL TIM_CHANNEL_4
/* Hall Effect Setups */
#define DISTANCE_BETWEEN_WHEEL_MAGS_CM 5.7
#define DISTANCE_BETWEEN_PEDAL_MAGS_CM 3.5
/* Timing for System */
#define TIM3_FREQUENCY_HZ 10000.0
#define TIM3_TIME_PER_TICK (1/TIM3_FREQUENCY_HZ)
#define SYSTEM_TIME_DELTA_MS 100.0
#define SYSTEM_TIME_MS_TO_TIM3(sys_time) (sys_time / (TIM3_TIME_PER_TICK * 1000.0))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
hall_effect_sensor_t frnt_wheel;
hall_effect_sensor_t back_wheel;
hall_effect_sensor_t pedal_gear;

MPU6050_t mpu_dev;

kalman_t kalman_x = {
        .q_angle = 0.001,
        .q_bias = 0.003,
        .r_measure = 0.03
};

kalman_t kalman_y = {
        .q_angle = 0.001,
        .q_bias = 0.003,
        .r_measure = 0.03,
};

alpha_beta_t alpha_beta_x = {
		.alpha = 0.005,
		.beta = 0.85
};

alpha_beta_t alpha_beta_y = {
		.alpha = 0.005,
		.beta = 0.85
};

double complementary_angle_x = 0;
double complementary_angle_y = 0;

double ab_comp_angle_x = 0;
double ab_comp_angle_y = 0;

double bike_vel_kmh;
double pedal_vel_kmh;
uint32_t total_travelled_meters;
uint32_t frnt_wheel_travelled_meters;
uint32_t back_wheel_travelled_meters;

double dt_ms = SYSTEM_TIME_DELTA_MS;
double dt_s = SYSTEM_TIME_DELTA_MS / 1000.0;

uint8_t wait_flag = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void compute_dynamic_tilt();
void compute_heffects();

void compute_kalman_angles(double dt);
void compute_alpha_beta_angles(double dt);
void compute_complementary_angles();
void compute_ab_comp_angles();
void zero_out_angles();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* ID = TIMX * TIM_CHANNEL_MAX(6) + TIM_CHANNEL_X */
	frnt_wheel.id = get_hall_effect_sensor_id(FRONT_WHEEL_TIMX, FRONT_WHEEL_CHL);
	back_wheel.id = get_hall_effect_sensor_id(BACK_WHEEL_TIMX, BACK_WHEEL_CHL);
	pedal_gear.id = get_hall_effect_sensor_id(PEDAL_GEAR_TIMX, PEDAL_GEAR_CHL);
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	configure_hall_effect(&frnt_wheel);
	configure_hall_effect(&back_wheel);
	configure_hall_effect(&pedal_gear);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_UART4_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	htim3.Instance->ARR = SYSTEM_TIME_MS_TO_TIM3(dt_ms);

//	l3gd20_init(&hspi2);
//	lsm303c_init(&hspi2);
	while (MPU6050_Init(&hi2c1) == 1);
	HAL_TIM_IC_Start_DMA(&(FRONT_WHEEL_TIM), FRONT_WHEEL_CHL, frnt_wheel.buffer, frnt_wheel.buf_len);
	HAL_TIM_IC_Start_DMA(&(BACK_WHEEL_TIM), BACK_WHEEL_CHL, back_wheel.buffer, back_wheel.buf_len);
	HAL_TIM_IC_Start_DMA(&(PEDAL_GEAR_TIM), PEDAL_GEAR_CHL, pedal_gear.buffer, pedal_gear.buf_len);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	zero_out_angles();

	HAL_Delay(2000);
	HAL_TIM_Base_Start_IT(&htim3);
	char buffer[64] = { 0 };
  	size_t size;

	while (1)
	{
		wait_flag = 1;

		compute_dynamic_tilt();
		compute_heffects();

#if MY_DEBUG == 1
		size = snprintf(buffer, 64, "Acce: X: %0.4f\tY: %0.4f\n\r", mpu_dev.AccX, mpu_dev.AccY);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
		size = snprintf(buffer, 64, "Gyro: X: %0.4f\tY: %0.4f\tZ: %0.4f\n\r", mpu_dev.GyroX, mpu_dev.GyroY, mpu_dev.GyroZ);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);

		HAL_UART_Transmit(&huart2, (uint8_t *)"=================\n\r", sizeof("=================\n\r"), 0xFFFF);

		size = snprintf(buffer, 64, "Kalman: X: %0.4f\tY: %0.4f\n\r", kalman_x.angle, kalman_y.angle);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
		size = snprintf(buffer, 64, "Alpha: X: %0.4f\tY: %0.4f\n\r", alpha_beta_x.alpha_out, alpha_beta_y.alpha_out);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
		size = snprintf(buffer, 64, "Complementary: X: %0.4f\tY: %0.4f\n\r", complementary_angle_x, complementary_angle_y);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
		size = snprintf(buffer, 64, "Alpha + Comp: X: %0.4f\tY: %0.4f\n\r", ab_comp_angle_x, ab_comp_angle_y);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
#endif

		/* Consume for set time */
		while (wait_flag == 1);
#if MY_DEBUG == 1
		HAL_UART_Transmit(&huart2, (uint8_t *)"\033c", sizeof("\033c"), 0xFFFF);
#endif
		/* Read position via GPS */
		/* Read speed via GPS */
		/* Calculate acce via GPS */
		  /* acce = (speed_now - speed_before) / time */
		/* Read linear acceleration via IMU */
		  /* This requires hefty math */
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/** Computes the dynamic tilt of the device.
 *
 * Dynamic tilt takes into account the linear acceleration of the body.
 * As a result, a gyroscope AND accelerometer is used to determine the incline
 * of the bike as the user moves. It will mostly be looking for steady-state
 * incline as there is a lot of noise as a bike travels. The desired response
 * time to an incline is approximately ~2-3 seconds. This should provide enough
 * phase error when using an advanced filter such as a Kalman filter or
 * integrating a low pass filter for noise reduction.
 */
void compute_dynamic_tilt()
{
	/* Compute dynamic tilt angle via IMU */
	MPU6050_Read_All(&hi2c1, &mpu_dev);
	MPU6050_Compute_Angles(&mpu_dev, dt_s);
	compute_kalman_angles(dt_s);
	compute_alpha_beta_angles(dt_s);
	compute_complementary_angles();
	compute_ab_comp_angles();
}

/** Computes the angle of the device using a Kalman Filter.
 *
 * This filter is complicated and is mostly derived from online sources.
 *
 * Note: This filter is tuned well to remove a lot of steady state drift.
 * An additional filter (e.g low pass filter) should be implemented to help
 * with noise reduction.
 */
void compute_kalman_angles(double dt)
{
	if ((mpu_dev.AccY < -90 && kalman_y.angle > 90) || (mpu_dev.AccY > 90 && kalman_y.angle < -90))
	{
	  kalman_y.angle = mpu_dev.AccY;
	}
	else
	{
	  kalman_y.angle = kalman_get_angle(&kalman_y, mpu_dev.AccY, mpu_dev.Gy, dt);
	}

	if (fabs(kalman_y.angle) > 90)
	{
	  mpu_dev.Gx = -mpu_dev.Gx;
	}
	kalman_x.angle = kalman_get_angle(&kalman_x, mpu_dev.AccX, mpu_dev.Gx, dt);
}

/** Computes the angle of the device using an alpha-beta filter.
 *
 * The alpha-beta filter is extremely simple and only takes into account the
 * gyroscope. As such, this filter introduces a lot of long term drift but
 * handles noise fairly well compared to the Kalman Filter. It's response time
 * is also adequate.
 */
void compute_alpha_beta_angles(double dt)
{
	alpha_beta_update(&alpha_beta_x, mpu_dev.Gx, dt);
	alpha_beta_update(&alpha_beta_y, mpu_dev.Gy, dt);
}

/** Computes the angle of the device using a complementary angle.
 *
 * The complementary filter is also extremely simple but requires the angle to
 * be computed using Euler math. This causes a Gimble-lock along the yaw axis
 * which normally is an issue but for this case, only the pitch axis matters.
 *
 * Note: This filter needs to be tuned better as the long term drift isn't
 * completely mitigated by the accelerometer.
 */
void compute_complementary_angles()
{
	complementary_angle_x = 0.04f * mpu_dev.AccX + 0.96 * mpu_dev.GyroX;
	complementary_angle_y = 0.04f * mpu_dev.AccY + 0.96 * mpu_dev.GyroY;
}

/** Computes the angle of the device by combining the alpha-beta and complementary filters.
 *
 * This combines both the alpha-beta filter and the complementary filter together
 * to help cover some of their faults. It works for a bit but still exhibits long
 * term drift as the complimentary filter part isn't perfect yet.
 */
void compute_ab_comp_angles()
{
	ab_comp_angle_x = 0.04f * mpu_dev.AccX + 0.96 * alpha_beta_x.alpha_out;
	ab_comp_angle_y = 0.04f * mpu_dev.AccY + 0.96 * alpha_beta_y.alpha_out;
}

/** Zeros out the starting angles for the MPU6050
 *
 * This is used for initial calibration. Ideally, each MPU device will
 * know absolute flatness and will not significantly drift from that.
 *
 * Note: The MPU6050 device is being used in place of a inclino-meter as
 * those devices are expensive but extremely precise.
 *
 * Note: A magnetometer can be used in conjunction with the MPU6050 for
 * better precision and lower drift. The STM32L476VG has a built in MEMs
 * magnetometer but is a WIP.
 */
void zero_out_angles()
{
	char buffer[64] = { 0 };
	double error_temps[5] = { 0 };
	size_t size;

	for (int i = 0; i < 50; i++)
	{
		MPU6050_Read_All(&hi2c1, &mpu_dev);

		HAL_Delay(dt_ms);
	}

	for (int i = 0; i < 50; i++)
	{
		MPU6050_Read_All(&hi2c1, &mpu_dev);
		MPU6050_Compute_Angles(&mpu_dev, dt_s);

		error_temps[0] += mpu_dev.AccX;
		error_temps[1] += mpu_dev.AccY;

		error_temps[2] += mpu_dev.Gx;
		error_temps[3] += mpu_dev.Gy;
		error_temps[4] += mpu_dev.Gz;

		HAL_Delay(dt_ms);
	}

	mpu_dev.AccXError -= error_temps[0] / 50.0;
	mpu_dev.AccYError -= error_temps[1] / 50.0;

	mpu_dev.GyroXError -= error_temps[2] / 50.0;
	mpu_dev.GyroYError -= error_temps[3] / 50.0;
	mpu_dev.GyroZError -= error_temps[4] / 50.0;

	size = snprintf(buffer, 64, "Acc Zero: X: %0.4f\tY: %0.4f\n\r", mpu_dev.AccXError, mpu_dev.AccYError);
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
	size = snprintf(buffer, 64, "Gyro Zero: X: %0.4f\tY: %0.4f\tZ: %0.4f\n\r", mpu_dev.GyroXError, mpu_dev.GyroYError, mpu_dev.GyroZError);
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, size, 0xFFFF);
}

/** Perform calculations using the hall effect sensor hardware
 *
 * Hall effects are used to calculate:
 *  - Total distance traveled (ticks * distance_between_mags)
 *  - Velocity (distance / time_between_mags)
 *  - Acceleration (WIP)
 *
 * This sensor is expected to have little to no noise. Possibly
 * some de-bouncing is required for switch triggers though a
 * Schmitt trigger is already integrated into the sensors
 *
 * Note: The total distance traveled is absolute. The ticks counter will
 * update continuously regardless of this function call. The distance will
 * never overflow as it will require the user to travel over ~151000 miles
 * (5.7cm * (2^32 - 1)).
 *
 * Note: Velocity is a direct calculation but assumes constant movement.
 * This means that the velocity may need a diff max to assume 0 velocity.
 * @Current Setup: (Average walking speed is 5 km/h [https://en.wikipedia.org/wiki/Walking])
 * - Slowest Speed: 0.00004777684809 km/h
 * - Fastest Speed: 205200 km/h
 *
 * Note: Acceleration will require a higher order filter such as an alpha-beta
 * filter as it will be a derivation of the calculated velocity.
 */
void compute_heffects()
{
	double f_whl_vel;
	double b_whl_vel;
	double p_vel;

	double f_dist_trav;
	double b_dist_trav;

	/* Simple velocity calculator (distance / time) */
	/*
	 *                   1 m       1 km
	 * D_B_W_MAGS cm * ------- * ---------
	 *                  100 cm    1000 m
	 * ----------------------------------------
	 *  diff ticks/s      1 min       1 hr
	 * -------------- * --------- * ---------
	 *  freq Hz           60 s        60 min
	 */
	f_whl_vel = (DISTANCE_BETWEEN_WHEEL_MAGS_CM / (100.0 * 1000.0))
			/ ((frnt_wheel.diff / TIM3_FREQUENCY_HZ) * (1.0/60.0) * (1.0/60.0));
	b_whl_vel = (DISTANCE_BETWEEN_WHEEL_MAGS_CM / (100.0 * 1000.0))
			/ ((back_wheel.diff / TIM3_FREQUENCY_HZ) * (1.0/60.0) * (1.0/60.0));
	p_vel = (DISTANCE_BETWEEN_PEDAL_MAGS_CM / (100.0 * 1000.0))
			/ ((pedal_gear.diff / TIM3_FREQUENCY_HZ) * (1.0/60.0) * (1.0/60.0));

	/* Compute distance traveled (ticks * distance) */
	/*
	 *                                      1 m
	 * frnt_wheel_ticks * D_B_W_MAGS cm * --------
	 *                                     100 cm
	 */
	f_dist_trav = frnt_wheel.ticks * DISTANCE_BETWEEN_WHEEL_MAGS_CM / 100.0;
	b_dist_trav = back_wheel.ticks * DISTANCE_BETWEEN_WHEEL_MAGS_CM / 100.0;

	/* Compute acceleration (WIP) */

	/* Save to system */
	bike_vel_kmh = (f_whl_vel + b_whl_vel) / 2.0;
	pedal_vel_kmh = p_vel;
	total_travelled_meters += (f_dist_trav + b_dist_trav) / 2.0;
	frnt_wheel_travelled_meters = f_dist_trav;
	back_wheel_travelled_meters = b_dist_trav;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		/* System failed to meet timing specifications */
		if (wait_flag == 0)
			Error_Handler();
		else
			wait_flag = 0;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

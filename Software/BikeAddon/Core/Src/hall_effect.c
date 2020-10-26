#include "hall_effect.h"
#include "common.h"

enum Callback_Type
{
	FULL,
	HALF
};

hall_effect_sensor_t* configured_h_effect_sensors[MAX_NUMBER_OF_SENSORS] = { NULL };

void hall_effect_callback(uint32_t sensor_id, uint32_t callback_type);

uint32_t get_hall_effect_sensor_id(uint32_t timx, uint32_t channel)
{
	return timx * TIM_CHANNEL_MAX + bit_position_to_int(channel);
}

void configure_hall_effect(hall_effect_sensor_t* sensor)
{
	int i;
	hall_effect_sensor_t** configured_sensor;
	for (i = 0; i < MAX_NUMBER_OF_SENSORS; i++)
	{
		configured_sensor = &(configured_h_effect_sensors[i]);
		if (*configured_sensor == NULL)
		{
			*configured_sensor = sensor;
		}
	}
	/* TODO: No more space :( */
}

void remove_hall_effect(hall_effect_sensor_t* sensor)
{
	int i;
	hall_effect_sensor_t** configured_sensor;
	for (i = 0; i < MAX_NUMBER_OF_SENSORS; i++)
	{
		configured_sensor = &(configured_h_effect_sensors[i]);
		if (*configured_sensor == sensor)
		{
			*configured_sensor = NULL;
		}
	}
	/* TODO: DNE, dont really care */
}

void hall_effect_callback(uint32_t sensor_id, uint32_t callback_type)
{
	int i;
	hall_effect_sensor_t* sensor;
	for (i = 0; i < MAX_NUMBER_OF_SENSORS; i++)
	{
		sensor = configured_h_effect_sensors[i];
		if (sensor != NULL)
		{
			if (sensor->id == sensor_id)
			{
				/* TODO: assert if sensor->id != sensor_id */
				switch(callback_type)
				{
				case FULL:
					sensor->diff = sensor->buffer[3] - sensor->buffer[2];
					break;
				case HALF:
					sensor->diff = sensor->buffer[1] - sensor->buffer[0];
					break;
				}
			}
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	int timx = 1;
	switch((unsigned long)(htim->Instance))
	{
	case (unsigned long)TIM2:
		timx++;
	case (unsigned long)TIM1:
		hall_effect_callback(get_hall_effect_sensor_id(timx, htim->Channel), FULL);
		break;
	default:
		/* No other TIM configured */
		break;
	}
}

void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	int timx = 1;
	switch((unsigned long)(htim->Instance))
	{
	case (unsigned long)TIM2:
		timx++;
	case (unsigned long)TIM1:
		hall_effect_callback(get_hall_effect_sensor_id(timx, htim->Channel), HALF);
		break;
	default:
		/* No other TIM configured */
		break;
	}
}

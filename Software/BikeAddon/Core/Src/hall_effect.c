#include "hall_effect.h"
#include "common.h"

enum Callback_Type
{
	FULL,
	HALF
};

hall_effect_sensor_t* configured_h_effect_sensors[MAX_NUMBER_OF_SENSORS] = { NULL };

void hall_effect_callback(uint32_t sensor_id, uint32_t callback_type);

void configure_hall_effect(hall_effect_sensor_t* sensor)
{
	configured_h_effect_sensors[sensor->id] = sensor;
}

void remove_hall_effect(hall_effect_sensor_t* sensor)
{
	configured_h_effect_sensors[sensor->id] = NULL;
}

void hall_effect_callback(uint32_t sensor_id, uint32_t callback_type)
{
	hall_effect_sensor_t* sensor = configured_h_effect_sensors[sensor_id];
	if (sensor != NULL)
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	hall_effect_callback(bit_position_to_int(htim->Channel), FULL);
}

void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	hall_effect_callback(bit_position_to_int(htim->Channel), HALF);
}

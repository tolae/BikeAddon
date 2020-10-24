#ifndef HALL_EFFECT_H
#define HALL_EFFECT_H

#include "tim.h"

#define MAX_NUMBER_OF_SENSORS (4)

#define HALL_EFFECT_SENSOR_BUFFER_MAX (4)

typedef struct hall_effect_sensor
{
	/* The sensor channel id this sensor is attached to */
	uint32_t id;
	/* The buffer to store the DMA data.
	 * If data size < 32, then the buffer size doesn't matter
	 * If data size > 32, change the buffer size to match as DMA caps at 32
	 */
	uint32_t buffer[HALL_EFFECT_SENSOR_BUFFER_MAX];
	/* The buffer length */
	uint16_t buf_len;
	/* The difference between two consecutive reads */
	uint32_t diff;
} hall_effect_sensor_t;

void configure_hall_effect(hall_effect_sensor_t* sensor);

#endif

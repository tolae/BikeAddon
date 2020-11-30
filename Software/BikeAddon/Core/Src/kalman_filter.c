/*
 * kalman_filter.c
 *
 *  Created on: Nov 29, 2020
 *      Author: tyson
 */

#include "kalman_filter.h"

double kalman_get_angle(kalman_t* kalman, double new_angle, double new_rate, double dt)
{
	double K[2];
	double S;
	double y;
	double rate;
	double P00_temp;
	double P01_temp;

	rate = new_rate - kalman->bias;
	kalman->angle += dt * rate;

	kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += kalman->q_bias * dt;

	S = kalman->P[0][0] + kalman->r_measure;
	K[0] = kalman->P[0][0] / S;
	K[1] = kalman->P[1][0] / S;

	y = new_angle - kalman->angle;
	kalman->angle += K[0] * y;
	kalman->bias += K[1] * y;

	P00_temp = kalman->P[0][0];
	P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;

	return kalman->angle;
}

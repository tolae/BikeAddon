/*
 * kelman_filter.h
 *
 *  Created on: Nov 29, 2020
 *      Author: tyson
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

typedef struct
{
	/* Kalman Filter Tunable Parameters */
	double q_angle;
	double q_bias;
	double r_measure;

	/* Outputs */
	double angle;
	double bias;

	/* Covariance Matrix */
	double P[2][2];
} kalman_t;

double kalman_get_angle(kalman_t* kalman, double new_angle, double new_rate, double dt);

#endif /* INC_KALMAN_FILTER_H_ */

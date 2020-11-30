/*
 * alpha_beta_filter.c
 *
 *  Created on: Nov 29, 2020
 *      Author: tyson
 */

#include "alpha_beta_filter.h"

int alpha_beta_update(alpha_beta_t* ab_filter, double input, double dt)
{
	double error;

	ab_filter->alpha_out = ab_filter->alpha_prev + (ab_filter->beta_prev * dt);
	ab_filter->beta_out = ab_filter->beta_prev;

	error = input - ab_filter->beta_out;

	ab_filter->alpha_out += ab_filter->alpha * error * dt;
	ab_filter->beta_out += (ab_filter->beta * error);

	ab_filter->alpha_prev = ab_filter->alpha_out;
	ab_filter->beta_prev = ab_filter->beta_out;

	return 0;
}

#ifndef ALPHA_BETA_FILTER_H
#define ALPHA_BETA_FILTER_H

typedef struct
{
	/* Tunable parameters */
	double alpha;
	double beta;

	/**Outputs
	 *
	 * These are the states associated with the parameters above.
	 */
	double alpha_out;
	double beta_out;

	/* Previous Values */
	double alpha_prev;
	double beta_prev;
} alpha_beta_t;

int alpha_beta_update(alpha_beta_t* filter_params, double input, double dt);

#endif

#ifndef StatePrinter_H
#define StatePrinter_H

#include <Eigen/Dense>
#include "ExtendedKalmanFilterIS1D.h"

namespace StatePrinter
{
	//file
	void print_states(char *filename, Eigen::Vector4d *states, int N);
	void print_signal(char *filename, double *signal, int N);
	void print_full_Kalman_states(char *filename, ExtendedKalmanFilterIS1DState *states, int N);

	//console
	void console_print_full_Kalman_state(ExtendedKalmanFilterIS1DState &state);
}

#endif

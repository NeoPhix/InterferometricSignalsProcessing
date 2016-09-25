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
	void print_Kalman_stdev(char *filename, Eigen::Vector4d *states, double *signal, double *noise,
		double *background, double *amplitude, double *frequency, double *phase, double *restoredSignal, int N);

	//console
	void console_print_full_Kalman_state(ExtendedKalmanFilterIS1DState &state);
	void console_print_Kalman_stdev(Eigen::Vector4d *states, double *signal, double *noise,
		double *background, double *amplitude, double *frequency, double *phase, double *restoredSignal, int N);
}

#endif

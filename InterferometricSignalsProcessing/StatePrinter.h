#ifndef StatePrinter_H
#define StatePrinter_H

#include <Eigen/Dense>
#include "ExtendedKalmanFilterIS1D.h"

class StatePrinter
{
public:
	void static print_states(char *filename, Eigen::Vector4d *states, int N);
	void static print_signal(char *filename, double *signal, int N);
	void static print_full_Kalman_states(char *filename, ExtendedKalmanFilterIS1DState *states, int N);
};

#endif

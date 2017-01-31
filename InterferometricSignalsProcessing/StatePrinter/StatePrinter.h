#ifndef StatePrinter_H
#define StatePrinter_H

#include <vector>

#include <Eigen/Dense>
#include "../DataModel/SignalAnalysis.h"
#include "../Filters/ExtendedKalmanFilterIS1D.h"

namespace printer
{
	//file
	void print_states( const char *filename, std::vector<Eigen::Vector4d> &states );

	void print_states( const char *filename, 
					   dmod::array1d &background, 
					   dmod::array1d &amplitude, 
					   dmod::array1d &frequency, 
					   dmod::array1d &phase );

	void print_signal( const char *filename, dmod::array1d &signal );

	void print_full_Kalman_states(const char *filename, std::vector<ExtendedKalmanFilterIS1DState> &states);

	void print_Kalman_stdev( const char *filename, 
							 std::vector<Eigen::Vector4d> &states, 
							 dmod::array1d &signal, 
							 dmod::array1d &noise,
							 dmod::array1d &background, 
							 dmod::array1d &amplitude, 
							 dmod::array1d &frequency, 
							 dmod::array1d &phase, 
							 dmod::array1d &restoredSignal );

	void print_cov_matrices( const char *filename, std::vector<ExtendedKalmanFilterIS1DState> &states );

	//console
	void console_print_full_Kalman_state(ExtendedKalmanFilterIS1DState &state);

	void console_print_Kalman_stdev( std::vector<Eigen::Vector4d> &states, 
									 dmod::array1d &signal, 
									 dmod::array1d &noise,
									 dmod::array1d &background, 
									 dmod::array1d &amplitude, 
									 dmod::array1d &frequency, 
									 dmod::array1d &phase, 
									 dmod::array1d &restoredSignal );
}

#endif

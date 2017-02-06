#include <vector>
#include <fstream>
#include <iostream>

#include "StatePrinter.h"
#include "../DataModel/SignalAnalysis.h"

namespace printer
{

	void print_states(const char *filename, std::vector<Eigen::Vector4d> &states)
	{
		size_t N = states.size();
		std::ofstream out(filename);
		for (size_t i = 0; i < N; i++)
		{
			out << states[i](0) << "\t" << states[i](1) << "\t" << states[i](2) << "\t" << states[i](3) << std::endl;
		}
		out.close();
	}

	void print_states( const char *filename, 
					   dmod::array1d &background, 
					   dmod::array1d &amplitude, 
				       dmod::array1d &frequency, 
					   dmod::array1d &phase )
	{
		size_t N = background.size();
		if (amplitude.size() != N || frequency.size() != N || phase.size() != N)
		{
			std::cout << "Error. Arrays sizes are mismatched." << std::endl;
			return;
		}
		std::ofstream out(filename);
		for (size_t i = 0; i < N; i++)
		{
			out << background[i] << "\t" << amplitude[i] << "\t" << frequency[i] << "\t" << phase[i] << std::endl;
		}
		out.close();
	}

	void print_signal( const char *filename, dmod::array1d &signal )
	{
		size_t N = signal.size();
		std::ofstream out(filename);
		for (int i = 0; i < N; i++)
		{
			out << signal[i] << std::endl;
		}
		out.close();
	}

	void print_full_Kalman_states( const char *filename, std::vector<ExtendedKalmanFilterIS1DState> &states )
	{
		size_t N = states.size();
		std::ofstream out(filename);
		for (int i = 0; i < N; i++)
		{
			out << states[i].state(0) << "\t" << states[i].state(1) << "\t" << states[i].state(2) << "\t" << states[i].state(3) << std::endl << std::endl;

			out << states[i].R(0, 0) << " " << states[i].R(0, 1) << " " << states[i].R(0, 2) << " " << states[i].R(0, 3) << std::endl;
			out << states[i].R(1, 0) << " " << states[i].R(1, 1) << " " << states[i].R(1, 2) << " " << states[i].R(1, 3) << std::endl;
			out << states[i].R(2, 0) << " " << states[i].R(2, 1) << " " << states[i].R(2, 2) << " " << states[i].R(2, 3) << std::endl;
			out << states[i].R(3, 0) << " " << states[i].R(3, 1) << " " << states[i].R(3, 2) << " " << states[i].R(3, 3) << std::endl << std::endl;

			out << states[i].Rn << std::endl << std::endl;

			out << states[i].Rw(0, 0) << " " << states[i].Rw(0, 1) << " " << states[i].Rw(0, 2) << " " << states[i].Rw(0, 3) << std::endl;
			out << states[i].Rw(1, 0) << " " << states[i].Rw(1, 1) << " " << states[i].Rw(1, 2) << " " << states[i].Rw(1, 3) << std::endl;
			out << states[i].Rw(2, 0) << " " << states[i].Rw(2, 1) << " " << states[i].Rw(2, 2) << " " << states[i].Rw(2, 3) << std::endl;
			out << states[i].Rw(3, 0) << " " << states[i].Rw(3, 1) << " " << states[i].Rw(3, 2) << " " << states[i].Rw(3, 3) << std::endl << std::endl;
		}
		out.close();
	}

	void console_print_full_Kalman_state( ExtendedKalmanFilterIS1DState &state )
	{
		std::cout << state.state(0) << "\t" << state.state(1) << "\t" << state.state(2) << "\t" << state.state(3) << std::endl << std::endl;

		std::cout << state.R(0, 0) << " " << state.R(0, 1) << " " << state.R(0, 2) << " " << state.R(0, 3) << std::endl;
		std::cout << state.R(1, 0) << " " << state.R(1, 1) << " " << state.R(1, 2) << " " << state.R(1, 3) << std::endl;
		std::cout << state.R(2, 0) << " " << state.R(2, 1) << " " << state.R(2, 2) << " " << state.R(2, 3) << std::endl;
		std::cout << state.R(3, 0) << " " << state.R(3, 1) << " " << state.R(3, 2) << " " << state.R(3, 3) << std::endl << std::endl;

		std::cout << state.Rn << std::endl << std::endl;

		std::cout << state.Rw(0, 0) << " " << state.Rw(0, 1) << " " << state.Rw(0, 2) << " " << state.Rw(0, 3) << std::endl;
		std::cout << state.Rw(1, 0) << " " << state.Rw(1, 1) << " " << state.Rw(1, 2) << " " << state.Rw(1, 3) << std::endl;
		std::cout << state.Rw(2, 0) << " " << state.Rw(2, 1) << " " << state.Rw(2, 2) << " " << state.Rw(2, 3) << std::endl;
		std::cout << state.Rw(3, 0) << " " << state.Rw(3, 1) << " " << state.Rw(3, 2) << " " << state.Rw(3, 3) << std::endl << std::endl;
	}

	void print_Kalman_stdev( const char *filename, 
							 std::vector<Eigen::Vector4d> &states, 
							 dmod::array1d &signal, 
							 dmod::array1d &noise,
							 dmod::array1d &background, 
							 dmod::array1d &amplitude, 
							 dmod::array1d &frequency, 
							 dmod::array1d &phase, 
							 dmod::array1d &restoredSignal )
	{
		size_t N = states.size();
		if ( signal.size() != N		||
			 noise.size() != N		|| 
			 background.size() != N || 
			 amplitude.size() != N	|| 
			 frequency.size() != N	|| 
			 phase.size() != N		|| 
			 restoredSignal.size() != N )
		{
			std::cout << "Error. Arrays sizes are mismatched." << std::endl;
			return;
		}
		std::ofstream out(filename);
		dmod::array1d tmp(N);

		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](0) - background[i];
		}
		out << dmod::stdev(tmp) << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](1) - amplitude[i];
		}
		out << dmod::stdev(tmp) << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](2) - frequency[i];
		}
		out << dmod::stdev(tmp) << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](3) - phase[i];
		}
		out << dmod::stdev(tmp) << std::endl;

		tmp = dmod::sub(signal, restoredSignal);
		out << dmod::snr(signal, tmp) << std::endl;

		out.close();
	}

	void console_print_Kalman_stdev( std::vector<Eigen::Vector4d> &states, 
									 dmod::array1d &signal, 
									 dmod::array1d &noise,
									 dmod::array1d &background, 
									 dmod::array1d &amplitude, 
									 dmod::array1d &frequency, 
									 dmod::array1d &phase, 
									 dmod::array1d &restoredSignal)
	{
		size_t N = states.size();
		if ( signal.size() != N		|| 
			 noise.size() != N		|| 
			 background.size() != N || 
			 amplitude.size() != N	|| 
			 frequency.size() != N	|| 
			 phase.size() != N		|| 
			 restoredSignal.size() != N )
		{
			std::cout << "Error. Arrays sizes are mismatched." << std::endl;
			return;
		}
		dmod::array1d tmp(N);

		std::cout << "Standard deviations:" << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](0) - background[i];
		}
		std::cout << "Background error:\t" << dmod::stdev(tmp) << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](1) - amplitude[i];
		}
		std::cout << "Amplitude error:\t" << dmod::stdev(tmp) << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](2) - frequency[i];
		}
		std::cout << "Frequency error:\t" << dmod::stdev(tmp) << std::endl;
		for (int i = 0; i < N; i++)
		{
			tmp[i] = states[i](3) - phase[i];
		}
		std::cout << "Phase error:\t\t" << dmod::stdev(tmp) << std::endl;

		tmp = dmod::sub(signal, restoredSignal);
		std::cout << "Signal-to-noise ratio:\t" << dmod::snr(signal, tmp) << std::endl;
	}

	void print_cov_matrices(const char *filename, std::vector<ExtendedKalmanFilterIS1DState> &states)
	{
		std::ofstream out(filename);
		for (auto iter = states.begin(); iter != states.end(); ++iter)
		{
			for (int y = 0; y < 4; ++y)
			{
				for (int x = 0; x < 4; ++x)
				{
					out << iter->R(y, x) << "\t";
				}
			}
			out << std::endl;
		}
	}
}
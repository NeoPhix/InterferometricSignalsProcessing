#include <fstream>
#include "StatePrinter.h"

void StatePrinter::print_states(char *filename, Eigen::Vector4d *states, int N)
{
	std::ofstream out;
	out.open(filename);
	for (int i = 0; i < N; i++)
	{
		out << states[i](0) << "\t" << states[i](1) << "\t" << states[i](2) << "\t" << states[i](3) << std::endl;
	}
	out.close();
}

void StatePrinter::print_signal(char *filename, double *signal, int N)
{
	std::ofstream out;
	out.open(filename);
	for (int i = 0; i < N; i++)
	{
		out << signal[i] << std::endl;
	}
	out.close();
}

void StatePrinter::print_full_Kalman_states(char *filename, ExtendedKalmanFilterIS1DState *states, int N)
{
	std::ofstream out;
	out.open(filename);
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
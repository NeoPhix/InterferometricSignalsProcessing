#include <iostream>
#include <ctime>
#include <functional>
#include <cmath>
#include <random>
#include <fstream>

#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "EKFIneterferometricSignal1D.h"

void print_states(char *filename, Eigen::Vector4d *states, int N);
void print_signal(char *filename, double *signal, int N);
void print_full_Kalman_states(char *filename, ExtendedKalmanFilterIS1DState *states, int N);

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;
	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl;

	//Signal
	const int N = 1000;
	double delta_z = 1;

	//Amplitude parameters
	double E_max = 20;
	double z1 = 250;
	double z2 = 800;
	double sigma1 = 200;
	double sigma2 = 150;

	double background[N] = { 0 };
	double amplitude[N] = { 0 };
	double frequency[N] = { 0 };
	double phase[N] = { 0 };
	double signal[N] = { 0 };

	//Noise
	std::default_random_engine gen((unsigned int)time(NULL));
	double noiseSigma = 10 ;
	double noiseMean = 0 ;
	double noise[N] = { 0 };
	std::normal_distribution<double> distribution(noiseMean, noiseSigma) ;

	//Signal modeling
	for (int i = 0; i < N; i++)
	{
		noise[i] = distribution(gen) ;
		background[i] = 100;
		amplitude[i] = 50 + 0.05 * i * sin(0.01*i) ;
		amplitude[i] = 50 + E_max*exp(-((i - z1)*(i - z1)) / (sigma1*sigma1)) +
			E_max*exp(-((i - z2)*(i - z2)) / (sigma2*sigma2));
		frequency[N - i - 1] = 0.03 + 0.00015*i;
		//if (i < N / 2)
		//{
		//	frequency[i] = 0.025 + 0.00005*i;
		//	frequency[N - i - 1] = frequency[i];
		//}
	}
	signal[0] = background[0] + amplitude[0] * cos(phase[0]);// +noise[0];
	for (int i = 1; i < N; i++)
	{
		phase[i] = phase[i - 1] + 2 * M_PI*frequency[i] * delta_z;
		signal[i] = background[i] + amplitude[i] * cos(phase[i]);// +noise[i];
	}
	print_signal("out.txt", signal, N) ;
	//std::cin >> argc ; 

	//Kalman parameters
	Eigen::Vector4d beginState(100, 70, 0.05, 1);
	Eigen::Matrix4d Rw;
	Rw << 0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002 ;
	Eigen::Matrix4d Rw_start(Rw);
	double Rn = 5;

	// Creation of EKF
	EKFIneterferometricSignal1D EKF(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	//Estimation
	Eigen::Vector4d *states = new Eigen::Vector4d[N];
	ExtendedKalmanFilterIS1DState * full_states = new ExtendedKalmanFilterIS1DState[N] ;
	for (int i = 0; i < N; i++)
	{
		full_states[i] = EKF.getFullState() ;
		EKF.estimate(signal[i]);
		states[i] = EKF.getState();
	}

	//Output to file
	print_states("EKFdata.txt", states, N);
	print_full_Kalman_states("EKF_fullstates.txt", full_states, N) ;

	//Output real data to file
	Eigen::Vector4d *real_states = new Eigen::Vector4d[N];
	for (int i = 0; i < N; i++)
	{	
		real_states[i](0) = 100;
		real_states[i](1) = amplitude[i];
		real_states[i](2) = frequency[i];
		real_states[i](3) = phase[i];
	}
	print_states("data.txt", real_states, N);

	//Memory release
	delete[] states;
	delete[] real_states ;
	delete[] full_states ;

	return 0;
}

void print_states(char *filename, Eigen::Vector4d *states, int N)
{
	std::ofstream out ;
	out.open(filename) ;
	for (int i = 0; i < N; i++)
	{
		out << states[i](0) << "\t" << states[i](1) << "\t" << states[i](2) << "\t" << states[i](3) << std::endl;
	}
	out.close() ;
}

void print_signal(char *filename, double *signal, int N)
{
	std::ofstream out;
	out.open(filename);
	for (int i = 0; i < N; i++)
	{
		out << signal[i] << std::endl;
	}
	out.close();
}

void print_full_Kalman_states(char *filename, ExtendedKalmanFilterIS1DState *states, int N)
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
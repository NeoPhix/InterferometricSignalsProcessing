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

void print_states(char *filename, Eigen::Vector4d *states, int N) ;


int main(int argc, char **argv)
{
	std::cout << argc << std::endl;
	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl;

	//Signal
	const int N = 1000;
	double delta_z = 1;
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
		amplitude[i] = 50 + E_max*exp(-((i - z1)*(i - z1)) / (sigma1*sigma1)) +
			E_max*exp(-((i - z2)*(i - z2)) / (sigma2*sigma2));
		frequency[N - i - 1] = 0.03 + 0.00015*i;
	}
	for (int i = 1; i < N; i++)
	{
		phase[i] = phase[i - 1] + 2 * M_PI*frequency[i] * delta_z;
		signal[i] = background[i] + amplitude[i] * cos(phase[i]) + noise[i];
	}

	//std::cin >> argc ; 

	//Kalman parameters
	Eigen::Vector4d beginState(100, 70, 0.05, 1);
	double tmp[] = { 0.1, 0, 0, 0,
					0, 0.15, 0, 0,
					0, 0, 0.0001, 0,
					0, 0, 0, 0.002 };
	Eigen::Matrix4d Rw(tmp);
	Eigen::Matrix4d Rw_start(tmp);
	double Rn = 5;

	// Creation of EKF
	EKFIneterferometricSignal1D EKF(beginState, Rw_start, Rw, Rn);

	//Estimation
	Eigen::Vector4d *states = new Eigen::Vector4d[N];
	for (int i = 0; i < N; i++)
	{
		EKF.estimate(signal[i]);
		states[i] = EKF.getState();
	}

	//Output to file
	print_states("EKFdata.txt", states, N);

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

	return 0;
}

void print_states(char *filename, Eigen::Vector4d *states, int N)
{
	std::ofstream out ;
	out.open(filename) ;
//	out << "Back\tAmp\tFreq\tPhase" << std::endl;
	for (int i = 0; i < N; i++)
	{
		out << states[i](0) << "\t" << states[i](1) << "\t" << states[i](2) << "\t" << states[i](3) << std::endl;
	}
	out.close() ;
}

#include <iostream>
#include <functional>
#include <cmath>
#include <fstream>

#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "SignalMaker.h"
#include "ExtendedKalmanFilterIS1D.h"
#include "StatePrinter.h"

const int N = 1000;
const double delta_z = 1;

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;
	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl;

	//Signal
	double E_max = 20;
	double z1 = 250;
	double z2 = 800;
	double sigma1 = 200;
	double sigma2 = 150;

	double background[N] = { 0 };
	double amplitude[N] = { 0 };
	double frequency[N] = { 0 };

	//Signal modeling
	for (int i = 0; i < N; i++)
	{
		background[i] = 100;
		amplitude[i] = 50 + E_max*SignalMaker::gaussianAmplitude(i, z1, sigma1) + E_max*SignalMaker::gaussianAmplitude(i, z2, sigma2);
		frequency[N - i - 1] = 0.03 + 0.00015*i;
	}


	double *phase = SignalMaker::phaseFromFrequency(0.03, N, delta_z);
	double *noise = SignalMaker::normalDistribution(0, 10, N) ;
	double *signal = SignalMaker::createSignal1D(background, amplitude, phase, noise, N, delta_z) ;

	StatePrinter::print_signal("out.txt", signal, N) ;

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
	ExtendedKalmanFilterIS1D EKF(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	//Estimation
	Eigen::Vector4d *states = new Eigen::Vector4d[N];
	ExtendedKalmanFilterIS1DState * full_states = new ExtendedKalmanFilterIS1DState[N] ;
	for (int i = 0; i < N; i++)
	{
		full_states[i] = EKF.getFullState();
		EKF.estimate(signal[i]);
		states[i] = EKF.getState();
	}

	//Output to file
	StatePrinter::print_states("EKFdata.txt", states, N);
	StatePrinter::print_full_Kalman_states("EKF_fullstates.txt", full_states, N) ;

	//Output real data to file
	Eigen::Vector4d *real_states = new Eigen::Vector4d[N];
	for (int i = 0; i < N; i++)
	{	
		real_states[i](0) = 100;
		real_states[i](1) = amplitude[i];
		real_states[i](2) = frequency[i];
		real_states[i](3) = phase[i];
	}
	StatePrinter::print_states("data.txt", real_states, N);

	//Memory release
	delete[] noise;
	delete[] phase;

	//States
	delete[] states;
	delete[] real_states;
	delete[] full_states;

	return 0;
}


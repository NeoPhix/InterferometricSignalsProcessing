#include <iostream>
#include <functional>
#include <cmath>
#include <fstream>
#include <ctime>
#include <random>

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

	//Signals
	int sigCount = 10;
	double E_max = 20;
	double sigma = 150;
	
	double background[N] = { 0 };
	double amplitude[N] = { 0 };
	double frequency[N] = { 0 };
	
	for (int i = 0; i < N; i++)
	{
		background[i] = 100;
		frequency[N - i - 1] = 0.03 + 0.00015*i;
	}

	double **signals = new double*[sigCount];
	std::default_random_engine gen((unsigned int)time(NULL));
	for (int k = 0; k < sigCount; k++)
	{
		signals[k] = new double[N];
		double startPhase = (double)(gen() % 100000000) / 100000000 * 2 * M_PI;
		std::cout << startPhase << std::endl ;
		double *phase = SignalMaker::phaseFromFrequency(frequency, startPhase, N, delta_z);
		double *noise = SignalMaker::normalDistribution(0, 10, N);
		
		int edgesCount = gen() % 4 + 1;		//Count of gaussian amplitudes in out signal
		for (int i = 0; i < N; i++)
		{
			amplitude[i] = 50 ;
			for (int j = 0; j < edgesCount; j++)
			{
				int z = gen() % 20 * 50 ;	//Now it is edge!
				amplitude[i] += E_max*SignalMaker::gaussianAmplitude(i, z, sigma) ;
			}
		}

		signals[k] = SignalMaker::createSignal1D(background, amplitude, phase, noise, N, delta_z) ; 
		StatePrinter::print_signal("out.txt", signals[k], N);
		
		delete[] phase;
		delete[] noise;
	}

	double *phase = SignalMaker::phaseFromFrequency(frequency, 0, N, delta_z);
	double *noise = SignalMaker::normalDistribution(0, 10, N);
	double *signal = SignalMaker::createSignal1D(background, amplitude, phase, noise, N, delta_z) ;

	StatePrinter::print_signal("out.txt", signal, N) ;

	

	// Creation of EKF
	Eigen::Vector4d beginState(100, 70, 0.05, 1);
	Eigen::Matrix4d Rw;
	Rw << 0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002 ;
	Eigen::Matrix4d Rw_start(Rw);
	double Rn = 5;

	ExtendedKalmanFilterIS1D EKF(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	//Estimation
	Eigen::Vector4d *states = new Eigen::Vector4d[N];
	for (int i = 0; i < N; i++)
	{
		EKF.estimate(signal[i]);
		states[i] = EKF.getState();
	}


	//Output to file
	StatePrinter::print_states("EKFdata.txt", states, N);

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
	for (int i = 0; i < sigCount; i++)
	{
		delete[] signals[i];
	}
	delete[] signals;

	//
	delete[] noise;
	delete[] phase;
	delete[] signal;

	//States
	delete[] states;
	delete[] real_states;

	return 0;
}


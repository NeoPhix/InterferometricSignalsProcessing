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
#include "SignalAnalysis.h"
#include "TotalSearchTuner.h"

const int N = 1000;
const double delta_z = 1;

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;
	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl;

	//Signals
	int sigCount = 10;
	double E_max = 50;
	double sigma = 50;
	
	double background[N] = { 0 };
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
		double startPhase = (double)(gen() % 100000000) / 100000000 * 2 * M_PI;
		double *phase = SignalMaker::phaseFromFrequency(frequency, startPhase, N, delta_z);
		double *noise = SignalMaker::normalDistribution(0, 10, N, gen);
		double *amplitude = SignalMaker::randomGaussianAmplitude(N, E_max, 100, sigma, 6, gen) ;

		signals[k] = SignalMaker::createSignal1D(background, amplitude, phase, noise, N) ;

		delete[] phase;
		delete[] noise;
		delete[] amplitude;
	}

	double *amplitude = SignalMaker::randomGaussianAmplitude(N, 20, 70, sigma, 6, gen);
	double *phase = SignalMaker::phaseFromFrequency(frequency, 0, N, delta_z);
	double *noise = SignalMaker::normalDistribution(0, 10, N, gen);
	double *signal = SignalMaker::createSignal1D(background, amplitude, phase, noise, N);

	StatePrinter::print_signal("out.txt", signal, N) ;

	////Creation of EKF
	//Eigen::Vector4d beginState(100, 70, 0.05, 1);
	//Eigen::Matrix4d Rw;
	//Rw << 0.1, 0, 0, 0,
	//	0, 0.15, 0, 0,
	//	0, 0, 0.001, 0,
	//	0, 0, 0, 0.002 ;
	//Eigen::Matrix4d Rw_start(Rw);
	//double Rn = 5;
	//ExtendedKalmanFilterIS1D EKF(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	//Creation of tuned EKF
	ExtendedKalmanFilterIS1DState minimal;
	ExtendedKalmanFilterIS1DState maximal;

	minimal.state = Eigen::Vector4d(0, 30, 0.00, 0);
	minimal.Rw << 
		0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	minimal.R = minimal.Rw;
	minimal.Rn = 1 ;

	maximal.state = Eigen::Vector4d(255, 140, 0.15, 2*M_PI);
	maximal.Rw <<
		0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	maximal.R = maximal.Rw;
	maximal.Rn = 10;

	FilterTuning::TotalSearchTuner tuner(signals, N, sigCount, 1000, gen, minimal, maximal);
	tuner.createStates();
	ExtendedKalmanFilterIS1DState tunedParameters = tuner.tune() ;
	StatePrinter::console_print_full_Kalman_state(tunedParameters);
	ExtendedKalmanFilterIS1D EKF(tunedParameters);

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


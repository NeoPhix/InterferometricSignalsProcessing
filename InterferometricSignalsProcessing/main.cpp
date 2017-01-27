#include <iostream>
#include <functional>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <random>

#include <Eigen/Dense>

#include "StatePrinter/StatePrinter.h"

#include "Filters/ExtendedKalmanFilterIS1D.h"
#include "Filters/GradientDescentFilterIS1D.h"

#include "FilterTuning/TotalSearchTuner.h"
#include "FilterTuning/GradientTuner.h"
#include "FilterTuning/FilterTuning.h"

#include "DataModel/SignalMaker.h"
#include "DataModel/SignalAnalysis.h"
#include "DataModel/Tomogram.h"

//double** getLearningSignals(int sigCount, double *background, double *frequency, double E_min, double E_max, double sigma, double delta_z, const int N, std::default_random_engine &gen)
//{
//	double **signals = new double*[sigCount];
//	for (int k = 0; k < sigCount; k++)
//	{
//		double startPhase = (double)(gen() % 100000000) / 100000000 * 2 * M_PI;
//		double *phase = SignalMaker::phaseFromFrequency(frequency, startPhase, N, delta_z);
//		double *noise = SignalMaker::normalDistribution(0, 10, N, gen);
//		double *amplitude = SignalMaker::randomGaussianAmplitude(N, E_min, E_max, sigma, 6, gen);
//
//		signals[k] = SignalMaker::createarray1d(background, amplitude, phase, noise, N);
//
//		delete[] phase;
//		delete[] noise;
//		delete[] amplitude;
//	}
//	return signals;
//}
//
//ExtendedKalmanFilterIS1D getTunedKalman_TotalSearch(double **signals, const int N, int sigCount, std::default_random_engine &gen)
//{
//	//Creation of EKF tuned by TotalSearch
//	ExtendedKalmanFilterIS1DState minimal;
//	ExtendedKalmanFilterIS1DState maximal;
//
//	minimal.state = Eigen::Vector4d(0, 0, 0.00, 0);
//	minimal.Rw <<
//		0.1, 0, 0, 0,
//		0, 0.15, 0, 0,
//		0, 0, 0.001, 0,
//		0, 0, 0, 0.002;
//	minimal.R = minimal.Rw;
//	minimal.Rn = 0.1;
//
//	maximal.state = Eigen::Vector4d(255, 140, 0.158, 2 * M_PI);
//	maximal.Rw <<
//		0.1, 0, 0, 0,
//		0, 0.15, 0, 0,
//		0, 0, 0.001, 0,
//		0, 0, 0, 0.002;
//	maximal.R = maximal.Rw;
//	maximal.Rn = 10;
//
//	FilterTuning::TotalSearchTuner tuner(signals, N, sigCount, 10, gen, minimal, maximal);
//	tuner.createStates();
//	ExtendedKalmanFilterIS1DState tunedParameters = tuner.tune();
//	printer::console_print_full_Kalman_state(tunedParameters);
//	return ExtendedKalmanFilterIS1D(tunedParameters);
//}
//
//void estimate(ExtendedKalmanFilterIS1D &EKF, double *signal, Eigen::Vector4d *states, double *restoredSignal, int N)
//{
//	for (int i = 0; i < N; i++)
//	{
//		EKF.estimate(signal[i]);
//		states[i] = EKF.getState();
//		restoredSignal[i] = EKF.evaluateSignalValue();
//	}
//}
//
//ExtendedKalmanFilterIS1D getTunedKalman_Gradient(ExtendedKalmanFilterIS1DState begin, ExtendedKalmanFilterIS1DState step, 
//	double **signals, const int N, int sigCount, int iterationsCount)
//{
//	FilterTuning::GradientTuner tuner(signals, N, sigCount, iterationsCount, begin, step);
//	ExtendedKalmanFilterIS1DState tunedParameters = tuner.tune();
//	//printer::console_print_full_Kalman_state(tunedParameters);
//	return ExtendedKalmanFilterIS1D(tunedParameters);
//}

int main(int argc, char **argv)
{
	const int N = 500;				//Signals modeling
	const double delta_z = 1;		

	double E_min = 0;	//Max amplitude
	double E_max = 200;	//Max amplitude
	double sigma = 200;
	std::default_random_engine gen((unsigned int)time(NULL));

	dmod::array1d background(N);
	dmod::array1d amplitude(N);
	dmod::array1d frequency(N);

	for (int i = 0; i < N; i++)
	{
		background[i] = 100;
		//amplitude[i] = 50 - dmod::gaussianAmplitude(i, 250, sigma);
		frequency[i] = 0.105 - 0.00015*i;
	}

	//Tests

	////Estimated signal

	//dmod::array1d phase = dmod::phaseFromFrequency(frequency, 0, delta_z);
	//dmod::array1d noise = dmod::normalDistribution(0, 10, N, gen);
	//dmod::array1d signal = dmod::createarray1d(background, amplitude, phase, noise, N);
	//
	//printer::print_signal("out.txt", signal, N);
	//printer::print_states("data.txt", background, amplitude, frequency, phase, N);
	//std::cout << SignalAnalysis::snr(signal, noise, N) << std::endl;

	////test arrays
	//Eigen::Vector4d *states = new Eigen::Vector4d[N];
	//double *restoredSignal = new double[N];

	////Creation of EKF
	//Eigen::Vector4d beginState(100, 40, 0.17985, 0);
	//Eigen::Matrix4d Rw;
	//Rw << 0.1, 0, 0, 0,
	//	0, 0.15, 0, 0,
	//	0, 0, 0.0005, 0,
	//	0, 0, 0, 0.002;
	//double Rn = 0.5;

	//std::cin >> E_min;




	
	//
	////EKF
	//ExtendedKalmanFilterIS1D EKF(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);
	//for (int i = 0; i < N; i++)
	//{
	//	EKF.estimate(signal[i]);
	//	states[i] = EKF.getState();
	//	restoredSignal[i] = EKF.evaluateSignalValue();
	//}
	//printer::print_states("EKF_data.txt", states, N);
	//printer::print_Kalman_stdev("EKF_deviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal, N);

	////GD
	//Eigen::Vector4d step(0.02, 0.009, 0, 0.0005);
	//GradientDescentFilterIS1D GDF(beginState, step, 1000) ;
	//for (int i = 0; i < N; i++)
	//{
	//	GDF.estimate(signal[i], StopCriterion::AdaptiveStep, true);
	//	states[i] = GDF.getState();
	//	restoredSignal[i] = GDF.evaluateSignalValue();
	//}
	//printer::print_states("GD_data.txt", states, N);
	//printer::print_Kalman_stdev("GD_deviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal, N);

	////GD+EKF
	//GDF = GradientDescentFilterIS1D(beginState, step, 50);
	//EKF = ExtendedKalmanFilterIS1D(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);
	//for (int i = 0; i < N; i++)
	//{
	//	EKF.estimate(signal[i]);
	//	GDF.setState(EKF.getState());
	//	GDF.estimate(signal[i], StopCriterion::AdaptiveStep, false);
	//	states[i] = GDF.getState();
	//	restoredSignal[i] = GDF.evaluateSignalValue();
	//	EKF.setState(states[i]);
	//}
	//printer::print_states("EKF_GD_data.txt", states, N);
	//printer::print_Kalman_stdev("EKF_GD_deviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal, N);

	//return 0;
}


//Main with GD larning for BPO backup
//
//int main(int argc, char **argv)
//{
//	//Signals modeling
//	const int N = 500;
//	const double delta_z = 1;
//
//	int sigCount = 9;	//learning signals count
//	double E_min = 50;	//Max amplitude
//	double E_max = 100;	//Max amplitude
//	double sigma = 50;
//
//	double background[N];
//	double frequency[N];
//
//	for (int i = 0; i < N; i++)
//	{
//		background[i] = 130;
//		frequency[i] = 0.17985 - 0.0002*i;
//	}
//
//	//Learning signals
//	std::default_random_engine gen((unsigned int)time(NULL));
//	double **signals = getLearningSignals(sigCount, background, frequency, E_min, E_max, sigma, delta_z, N, gen);
//
//	//Estimated signal
//	int edges[3] = { 100, 200, 425 };
//	double *amplitude = SignalMaker::fixedGaussianAmplitude(N, E_min, E_max, sigma, edges, 3);
//	double *phase = SignalMaker::phaseFromFrequency(frequency, 0, N, delta_z);
//	double *noise = SignalMaker::normalDistribution(0, 10, N, gen);
//	double *signal = SignalMaker::createarray1d(background, amplitude, phase, noise, N);
//	printer::print_signal("out.txt", signal, N);
//	printer::print_states("data.txt", background, amplitude, frequency, phase, N);
//	std::cout << SignalAnalysis::snr(signal, noise, N) << std::endl;
//
//	//test arrays
//	Eigen::Vector4d *states = new Eigen::Vector4d[N];
//	double *restoredSignal = new double[N];
//
//	//No learning
//	//Creation of EKF
//	Eigen::Vector4d beginState(100, 70, 0.17985, 0);
//	Eigen::Matrix4d Rw;
//	Rw << 0.1, 0, 0, 0,
//		0, 0.15, 0, 0,
//		0, 0, 0.0005, 0,
//		0, 0, 0, 0.002;
//	double Rn = 0.5;
//	ExtendedKalmanFilterIS1D EKF(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);
//
//	printer::console_print_full_Kalman_state(EKF.getFullState());
//
//	estimate(EKF, signal, states, restoredSignal, N);
//	printer::print_states("EKFdata.txt", states, N);
//	printer::print_Kalman_stdev("EKFdeviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal, N);
//
//	//With GD-learning
//	ExtendedKalmanFilterIS1DState begin;
//	ExtendedKalmanFilterIS1DState step;
//
//	///////////
//	int K = 400;
//	double **result = new double*[K];
//
//	for (int x = 0; x < K; x++)
//	{
//		result[x] = new double[K];
//		for (int y = 0; y < K; y++)
//		{
//			begin.state = Eigen::Vector4d(100, x * 400 / double(K) - 200, 0.17985, y*6.28 / double(K));
//			begin.Rw <<
//				0.1, 0, 0, 0,
//				0, 0.15, 0, 0,
//				0, 0, 0.0005, 0,
//				0, 0, 0, 0.002;
//			begin.R = Eigen::Matrix4d::Identity();
//			begin.Rn = 0.5;
//			result[x][y] = FilterTuning::fitness(signals, sigCount, N, begin);
//		}
//	}
//
//	std::ofstream out("fitness.txt");
//	for (int x = 0; x < K; x++)
//	{
//		for (int y = 0; y < K; y++)
//		{
//			out << result[x][y] << " ";
//		}
//		out << std::endl;
//	}
//	out.close();
//	//////////////////
//
//	//Super tests
//	//1 only vector
//	int M = 1000;
//	Eigen::Vector4d *deviations = new Eigen::Vector4d[M];
//	Eigen::Vector4d *starts = new Eigen::Vector4d[M];
//
//	begin.state = Eigen::Vector4d(150, 21, 0.05, 3);
//	begin.Rw <<
//		0.1, 0, 0, 0,
//		0, 0.15, 0, 0,
//		0, 0, 0.0005, 0,
//		0, 0, 0, 0.002;
//	begin.R = Eigen::Matrix4d::Identity();
//	begin.Rn = 0.5;
//
//	step.state = Eigen::Vector4d(1, 1, 0.005, 0.1)*0.002;
//	step.Rw <<
//		0, 0, 0, 0,
//		0, 0, 0, 0,
//		0, 0, 0, 0,
//		0, 0, 0, 0;
//	step.R <<
//		0, 0, 0, 0,
//		0, 0, 0, 0,
//		0, 0, 0, 0,
//		0, 0, 0, 0;
//	step.Rn = 0;
//
//	for (int i = 5; i < 5005; i += 5)
//	{
//		int k = i / 5 - 1;
//		EKF = getTunedKalman_Gradient(begin, step, signals, N, sigCount, 5);
//		begin = EKF.getFullState();
//		starts[k] = begin.state;
//		estimate(EKF, signal, states, restoredSignal, N);
//		deviations[k] = SignalAnalysis::get_deviations(states, signal, noise, background, amplitude, frequency, phase, restoredSignal, N);
//
//		std::stringstream str;
//		str << "GD_1_" << i << ".txt\0";
//		printer::print_states(str.str().c_str(), states, N);
//
//		std::cout << i << std::endl;
//	}
//	printer::print_states("GD_1_deviations.txt", deviations, M);
//	printer::print_states("GD_1_starts.txt", starts, M);
//
//	//Memory release
//	for (int i = 0; i < sigCount; i++)
//	{
//		delete[] signals[i];
//	}
//	delete[] signals;
//
//	//Signal parameters
//	delete[] amplitude;
//	delete[] noise;
//	delete[] phase;
//	delete[] signal;
//	delete[] restoredSignal;
//
//	//States
//	delete[] states;
//
//	return 0;
//} //main
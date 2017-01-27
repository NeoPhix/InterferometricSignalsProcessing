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

std::vector<dmod::array1d> getLearningSignals(int signalsAmount, dmod::array1d &background, dmod::array1d &frequency, 
	double E_min, double E_max, double sigma, double delta_z, std::default_random_engine &gen)
{
	size_t N = background.size();
	if (frequency.size() != N)
	{
		std::cout << "Error. Arrays sizes are mismatched." << std::endl;
		return std::vector<dmod::array1d>(0);
	}
	std::vector<dmod::array1d> signals(signalsAmount);
	for (int k = 0; k < signalsAmount; k++)
	{
		double startPhase = (double)(gen() % 100000000) / 100000000 * 2 * M_PI;
		dmod::array1d phase = dmod::phaseFromFrequency(frequency, startPhase, delta_z);
		dmod::array1d noise = dmod::createNormalNoise(0, 10, N, gen);
		dmod::array1d amplitude = dmod::randomGaussianAmplitude(static_cast<double>(N), E_min, E_max, sigma, 6, gen);

		signals[k] = dmod::createSignal1D(background, amplitude, phase, noise);
	}
	return signals;
}

EKF getTunedKalman_TotalSearch(std::vector<dmod::array1d> signals, std::default_random_engine &gen)
{
	//Creation of EKF tuned by TotalSearch
	EKFState minimal;
	EKFState maximal;

	minimal.state = Eigen::Vector4d(0, 0, 0.00, 0);
	minimal.Rw <<
		0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	minimal.R = minimal.Rw;
	minimal.Rn = 0.1;

	maximal.state = Eigen::Vector4d(255, 140, 0.158, 2 * M_PI);
	maximal.Rw <<
		0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	maximal.R = maximal.Rw;
	maximal.Rn = 10;

	FilterTuning::TotalSearchTuner tuner(signals, 10, gen, minimal, maximal);
	tuner.createStates();
	EKFState tunedParameters = tuner.tune();
	printer::console_print_full_Kalman_state(tunedParameters);
	return EKF(tunedParameters);
}

EKF getTunedKalman_Gradient(EKFState begin, EKFState step, std::vector<dmod::array1d> signals, int iterationsNumber)
{
	FilterTuning::GradientTuner tuner(signals, iterationsNumber, begin, step);
	EKFState tunedParameters = tuner.tune();
	return EKF(tunedParameters);
}

int main(int argc, char **argv)
{
	//OpenCV tests image loading



	const int N = 500;				//Signals modeling
	const double delta_z = 1;		

	double E_max = 200;	//Max amplitude
	double sigma = 200;
	std::default_random_engine gen((unsigned int)time(NULL));

	dmod::array1d background(N, 100);
	dmod::array1d amplitude(N, 50);
	dmod::array1d frequency(N);

	amplitude = dmod::sum(amplitude, dmod::fixedGaussianAmplitude(E_max, sigma, 250, N));

	for (int i = 0; i < N; i++)
	{
		frequency[i] = 0.105 - 0.00015*i;
	}

	//Estimated signal

	dmod::array1d phase = dmod::phaseFromFrequency(frequency, 0, delta_z);
	dmod::array1d noise = dmod::createNormalNoise(0, 10, N, gen);
	dmod::array1d signal = dmod::createSignal1D(background, amplitude, phase, noise);
	
	printer::print_signal("MatlabScripts/out.txt", signal);
	printer::print_states("MatlabScripts/data.txt", background, amplitude, frequency, phase);
	std::cout << dmod::snr(signal, noise) << std::endl;

	//Creation of EKF
	Eigen::Vector4d beginState(100, 40, 0.17985, 0);
	Eigen::Matrix4d Rw;
	Rw << 0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.0005, 0,
		0, 0, 0, 0.002;
	double Rn = 0.5;
	EKF filter(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);
	dmod::array1d restoredSignal = filter.getRestoredSignal(states);

	printer::print_states("MatlabScripts/EKF_data.txt", states);
	printer::print_Kalman_stdev("MatlabScripts/EKF_deviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal);

	return 0;
}

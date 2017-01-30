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
	float E_min, float E_max, float sigma, float delta_z, std::default_random_engine &gen)
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
		float startPhase = (float)(gen() % 100000000) / 100000000 * 2 * M_PI;
		dmod::array1d phase = dmod::phaseFromFrequency(frequency, startPhase, delta_z);
		dmod::array1d noise = dmod::createNormalNoise(0, 10, N, gen);
		dmod::array1d amplitude = dmod::randomGaussianAmplitude(static_cast<float>(N), E_min, E_max, sigma, 6, gen);

		signals[k] = dmod::createSignal1D(background, amplitude, phase, noise);
	}
	return signals;
}

EKF getTunedKalman_TotalSearch(std::vector<dmod::array1d> signals, size_t filtersAmount, std::default_random_engine &gen)
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

	maximal.state = Eigen::Vector4d(255, 5, 0.158, 2 * M_PI);
	maximal.Rw <<
		0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	maximal.R = maximal.Rw;
	maximal.Rn = 10;

	FilterTuning::TotalSearchTuner tuner(signals, filtersAmount, gen, minimal, maximal);
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
	std::default_random_engine gen((unsigned int)time(NULL));

	//OpenCV tests image loading							//TODO think about initial structure maybe
	dmod::Tomogram tomo;
	tomo.initSizeFromImageSequence("G:/data/21.01.2014/narost1/img", ".bmp", 1, 400);
	tomo.loadImageSequence("G:/data/21.01.2014/narost1/img", ".bmp", 1, 400);
	
	std::vector<dmod::array1d> learningData(15);
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	for (auto iter = learningData.begin(); iter != learningData.end(); ++iter)
	{
		size_t y = gen() % h;
		size_t x = gen() % w;
		*iter = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);
	}

	EKF filter = getTunedKalman_TotalSearch(learningData, 50, gen);

	dmod::array1d signal = tomo.getSignal1D(800, 350, 0, dmod::Axis::Z);
	std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);
	dmod::array1d restoredSignal = filter.getRestoredSignal(states);

	printer::print_states("MatlabScripts/EKF_real_data.txt", states);
	printer::print_signal("MatlabScripts/signal.txt", signal);
	printer::print_signal("MatlabScripts/restoredSignal.txt", restoredSignal);



	////Old signal processing
	//const int N = 500;				//Signals modeling
	//const float delta_z = 1;		

	//float E_max = 200;	//Max amplitude
	//float sigma = 200;


	//dmod::array1d background(N, 100);
	//dmod::array1d amplitude(N, 50);
	//dmod::array1d frequency(N);

	//amplitude = dmod::sum(amplitude, dmod::fixedGaussianAmplitude(E_max, sigma, N/2, N));

	//for (int i = 0; i < N; i++)
	//{
	//	frequency[i] = 0.105 - 0.00015*i;
	//}

	////Estimated signal
	//dmod::array1d phase = dmod::phaseFromFrequency(frequency, 0, delta_z);
	//dmod::array1d noise = dmod::createNormalNoise(0, 10, N, gen);
	//dmod::array1d signal = dmod::createSignal1D(background, amplitude, phase, noise);
	//
	//printer::print_signal("MatlabScripts/out.txt", signal);
	//printer::print_states("MatlabScripts/data.txt", background, amplitude, frequency, phase);
	//std::cout << dmod::snr(signal, noise) << std::endl;

	////Creation of EKF
	//Eigen::Vector4d beginState(100, 40, 0.17985, 0);
	//Eigen::Matrix4d Rw;
	//Rw << 0.1, 0, 0, 0,
	//	0, 0.15, 0, 0,
	//	0, 0, 0.0005, 0,
	//	0, 0, 0, 0.002;
	//float Rn = 0.5;
	//EKF filter(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	//std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);
	//dmod::array1d restoredSignal = filter.getRestoredSignal(states);

	//printer::print_states("MatlabScripts/EKF_data.txt", states);
	//printer::print_Kalman_stdev("MatlabScripts/EKF_deviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal);

	//return 0;
}

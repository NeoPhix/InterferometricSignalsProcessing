#include <iostream>
#include <functional>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <random>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> //to think why this headers

#include "StatePrinter/StatePrinter.h"

#include "Filters/ExtendedKalmanFilterIS1D.h"
#include "Filters/GradientDescentFilterIS1D.h"

#include "FilterTuning/TotalSearchTuner.h"
#include "FilterTuning/GradientTuner.h"
#include "FilterTuning/FilterTuning.h"

#include "DataModel/SignalMaker.h"
#include "DataModel/SignalAnalysis.h"
#include "DataModel/Tomogram.h"

std::vector<dmod::array1d> getLearningSignals( int signalsAmount, 
											   dmod::array1d &background, 
											   dmod::array1d &frequency, 
											   float E_min, 
											   float E_max, 
											   float sigma, 
											   float delta_z, 
											   std::default_random_engine &gen )
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
	return std::move(signals);
}

std::vector<dmod::array1d> getLearningSignals( int signalsAmount, 
											   dmod::Tomogram &tomo, 
											   std::default_random_engine &gen, 
											   cv::Mat &mask )
{
	std::vector<dmod::array1d> signals(signalsAmount);
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	for (auto iter = signals.begin(); iter != signals.end(); ++iter)
	{
		size_t y = gen() % h;
		size_t x = gen() % w;
		if (mask.ptr()[y * w + x] > 0)
		{
			*iter = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);
		}
		else
		{
			iter--;
		}
	}

	return std::move(signals);
}

std::vector<dmod::array1d> getLearningSignals( int signalsAmount, 
											   std::vector<dmod::array1d> input, 
											   std::default_random_engine &gen )
{
	std::vector<dmod::array1d> signals(signalsAmount);
	size_t n = input.size();

	for (auto iter = signals.begin(); iter != signals.end(); ++iter)
	{
		size_t x = gen() % n;
		*iter = input[x];
	}

	return std::move(signals);
}

std::vector<dmod::array1d> getLearningSignals( int signalsAmount, 
											   dmod::Tomogram &tomo, 
											   std::default_random_engine &gen )
{
	std::vector<dmod::array1d> signals(signalsAmount);
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	for (auto iter = signals.begin(); iter != signals.end(); ++iter)
	{
		size_t y = gen() % h;
		size_t x = gen() % w;
		*iter = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);
	}

	return signals;
}

EKFState getTunedKalmanState_TotalSearch( std::vector<dmod::array1d> signals, 
								size_t filtersAmount, 
								std::default_random_engine &gen )
{
	//Creation of EKF tuned by TotalSearch
	EKFState minimal;
	EKFState maximal;

	//minimal.state = Eigen::Vector4d(0, 0, 0.00, 0);
	//minimal.Rw <<
	//	0.01, 0, 0, 0,
	//	0, 0.01, 0, 0,
	//	0, 0, 0.001, 0,
	//	0, 0, 0, 0.002;
	//minimal.R = minimal.Rw;
	//minimal.Rn = 0.1;

	//maximal.state = Eigen::Vector4d(255, 10, 0.6, 2 * M_PI);
	//maximal.Rw <<
	//	0.4, 0, 0, 0,
	//	0, 0.4, 0, 0,
	//	0, 0, 0.1, 0,
	//	0, 0, 0, 0.2;
	//maximal.R = maximal.Rw;
	//maximal.Rn = 10;

	minimal.state = Eigen::Vector4d(0, 0, 0.00, 0);
	minimal.Rw <<
		0.1, 0, 0, 0,
		0, 0.15, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	minimal.R = minimal.Rw;
	minimal.Rn = 0.1;

	maximal.state = Eigen::Vector4d(255, 5, 0.5, 2 * M_PI);
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
	
	return tunedParameters;
}

EKFState getTunedKalmanState_Gradient( EKFState begin, 
							 EKFState step, 
							 std::vector<dmod::array1d> signals, 
							 int iterationsNumber )
{
	FilterTuning::GradientTuner tuner(signals, iterationsNumber, begin, step);
	EKFState tunedParameters = tuner.tune();
	return tunedParameters;
}

void sceanrioRealDataOCT_TotalSearch_ForBScan( const char *path,
											   const char *type,
										       size_t begin_number,
										       size_t end_number,
											   const char *maskname,
											   size_t signalsAmount,
											   size_t filtersAmount, 
											   std::default_random_engine &gen,
											   const char *outpath,
											   const char *outtype )
{
	dmod::Tomogram tomo;
	tomo.initSizeFromImageSequence(path, type, begin_number, end_number);
	tomo.loadImageSequence(path, type, begin_number, end_number);
	
	size_t d = tomo.getDepth();
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	dmod::Tomogram tomo_out(d, h, w);

	for (size_t y = 0; y < h; ++y)
	{
		std::vector<dmod::array1d> learningData = getLearningSignals( signalsAmount, tomo.getSignal2D(y, dmod::Plane::XZ), gen );
		EKFState tunedState = getTunedKalmanState_TotalSearch(learningData, filtersAmount, gen);
		for (size_t x = 0; x < w; ++x)
		{
			EKF filter(tunedState);
			dmod::array1d signal = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);
			std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);

			dmod::array1d amplitude = dmod::get_parameter_vector(states, AMPLITUDE);
			dmod::absolute(amplitude);
			dmod::normalize(amplitude, 0, 255);
			tomo_out.setSignal1D(x, y, 0, amplitude, dmod::Axis::Z);
		}
	}

	tomo_out.saveImageSequence(outpath, outtype, dmod::Plane::XZ);
}

void sceanrioRealDataOCT_TotalSearch( const char *path,
									  const char *type,
									  size_t begin_number,
									  size_t end_number,
									  const char *maskname,
									  size_t signalsAmount,
									  size_t filtersAmount,
									  std::default_random_engine &gen,
									  const char *outpath,
									  const char *outtype )
{
	dmod::Tomogram tomo;
	tomo.initSizeFromImageSequence(path, type, begin_number, end_number);
	tomo.loadImageSequence(path, type, begin_number, end_number);

	std::vector<dmod::array1d> learningData;
	cv::Mat mask = cv::imread(maskname, CV_LOAD_IMAGE_GRAYSCALE);

	if (mask.rows != 0)
		learningData = getLearningSignals(signalsAmount, tomo, gen, mask);
	else
		learningData = getLearningSignals(signalsAmount, tomo, gen);

	EKFState tunedState = getTunedKalmanState_TotalSearch(learningData, filtersAmount, gen);

	size_t d = tomo.getDepth();
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	dmod::Tomogram tomo_out(d, h, w);

	for (size_t y = 0; y < h; ++y)
	{
		for (size_t x = 0; x < w; ++x)
		{
			EKF filter(tunedState);
			dmod::array1d signal = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);
			std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);

			dmod::array1d amplitude = dmod::get_parameter_vector(states, AMPLITUDE);
			dmod::absolute(amplitude);
			dmod::normalize(amplitude, 0, 255);
			tomo_out.setSignal1D(x, y, 0, amplitude, dmod::Axis::Z);

		}
	}

	tomo_out.saveImageSequence(outpath, outtype, dmod::Plane::XZ);
	printer::print_full_Kalman_states("MatlabScripts/state.txt", std::vector<EKFState>(1, tunedState));
}

void sceanrioRealDataOCT_TotalSearch_Gradient( const char *path,
											   const char *type,
											   size_t begin_number,
											   size_t end_number,
											   const char *maskname,
											   size_t signalsAmount,
											   size_t filtersAmount,
											   std::default_random_engine &gen,
											   const char *outpath,
											   const char *outtype )
{
	dmod::Tomogram tomo;
	tomo.initSizeFromImageSequence(path, type, begin_number, end_number);
	tomo.loadImageSequence(path, type, begin_number, end_number);

	std::vector<dmod::array1d> learningData;
	cv::Mat mask = cv::imread(maskname, CV_LOAD_IMAGE_GRAYSCALE);

	if (mask.rows != 0)
		learningData = getLearningSignals(signalsAmount, tomo, gen, mask);
	else
		learningData = getLearningSignals(signalsAmount, tomo, gen);

	EKFState tunedState = getTunedKalmanState_TotalSearch(learningData, filtersAmount, gen);

	//Gradient
	EKFState step;

	step.state = Eigen::Vector4d(1, 0.1, 0.01, 0.1);
	step.Rw <<
		0.01, 0, 0, 0,
		0, 0.01, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	step.R = step.Rw;
	step.Rn = 0.1;
	
	tunedState = getTunedKalmanState_Gradient(tunedState, step, learningData, 100);

	//Output
	size_t d = tomo.getDepth();
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	dmod::Tomogram tomo_out(d, h, w);

	for (size_t y = 0; y < h; ++y)
	{
		for (size_t x = 0; x < w; ++x)
		{
			EKF filter(tunedState);
			dmod::array1d signal = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);
			std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);

			dmod::array1d amplitude = dmod::get_parameter_vector(states, AMPLITUDE);
			dmod::absolute(amplitude);
			dmod::normalize(amplitude, 0, 255);
			tomo_out.setSignal1D(x, y, 0, amplitude, dmod::Axis::Z);
		}
	}

	tomo_out.saveImageSequence(outpath, outtype, dmod::Plane::XZ);
	printer::print_full_Kalman_states("MatlabScripts/state.txt", std::vector<EKFState>(1, tunedState));
}


void scenarioModelSignalProcessing(std::default_random_engine &gen)
{
	const int N = 500;				//Signals modeling
	const float delta_z = 1;		

	float E_max = 200;			    //Max amplitude
	float sigma = 200;

	dmod::array1d background(N, 100);
	dmod::array1d amplitude(N, 50);
	dmod::array1d frequency(N);

	amplitude = dmod::sum(amplitude, dmod::fixedGaussianAmplitude(E_max, sigma, N/2, N));

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
	float Rn = 0.5;
	EKF filter(beginState, Eigen::Matrix4d::Identity(), Rw, Rn);

	std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);
	dmod::array1d restoredSignal = filter.getRestoredSignal(states);

	printer::print_states("MatlabScripts/EKF_data.txt", states);
	printer::print_Kalman_stdev("MatlabScripts/EKF_deviations.txt", states, signal, noise, background, amplitude, frequency, phase, restoredSignal);
}



int main( int argc, char **argv ) 
{
	std::default_random_engine gen((unsigned int)time(NULL));

	sceanrioRealDataOCT_TotalSearch( "D:/Data/ZhukovaSignals/onion1_resized/img", ".bmp", 1, 500,
									 "D:/Data/ZhukovaSignals/onion1_resized/mask.bmp",
									 15, 50000, gen,
									 "D:/Data/ZhukovaSignals/onion1_resized/output/", ".bmp");

	sceanrioRealDataOCT_TotalSearch( "D:/Data/ZhukovaSignals/onion2_resized/img", ".bmp", 1, 500,
									 "D:/Data/ZhukovaSignals/onion2_resized/mask.bmp",
									 15, 50000, gen,
									 "D:/Data/ZhukovaSignals/onion2_resized/output/", ".bmp");

	sceanrioRealDataOCT_TotalSearch( "D:/Data/ZhukovaSignals/egg1_resized/img", ".bmp", 1, 500,
									 "D:/Data/ZhukovaSignals/egg1_resized/mask.bmp",
									 15, 50000, gen,
									 "D:/Data/ZhukovaSignals/egg1_resized/output/", ".bmp");

	return 0;
}


//dmod::array1d restoredSignal = filter.getRestoredSignal(states);
//printer::print_states("MatlabScripts/EKF_real_data.txt", states);
//printer::print_signal("MatlabScripts/signal.txt", signal);
//printer::print_signal("MatlabScripts/restoredSignal.txt", restoredSignal);
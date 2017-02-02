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

/////////////////////
///getLearningData///
/////////////////////
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

/////////////////////
///Paintin///////////
/////////////////////
void paintBScans( const char *outpath, 
				  const char *outtype,
				  dmod::Tomogram &tomo, 
				  EKFState &tunedState, 
				  dmod::Plane plane )
{
	size_t d = tomo.getDepth();
	size_t h = tomo.getHeight();
	size_t w = tomo.getWidth();

	dmod::Tomogram tomo_out(d, h, w);

	for (size_t y = 0; y < h; ++y)
	{
		for (size_t x = 0; x < w; ++x)
		{
			dmod::array1d signal = tomo.getSignal1D(x, y, 0, dmod::Axis::Z);

			EKFState newState = tunedState;
			newState.state(BACKGROUND) = signal[0];
			newState.state(AMPLITUDE) = 0;
			EKF filter(newState);

			std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);

			dmod::array1d amplitude = dmod::get_parameter_vector(states, AMPLITUDE);
			dmod::absolute(amplitude);
			dmod::normalize(amplitude, 0, 255);
			if (dmod::mean(amplitude) > 100)
			{
				amplitude = dmod::array1d(signal.size(), 0);
			}
			tomo_out.setSignal1D(x, y, 0, amplitude, dmod::Axis::Z);


			dmod::array1d restoredSignal = filter.getRestoredSignal(states);
			printer::print_states("MatlabScripts/EKF_real_data.txt", states);
			printer::print_signal("MatlabScripts/signal.txt", signal);
			printer::print_signal("MatlabScripts/restoredSignal.txt", restoredSignal);
		}
	}



	//

	tomo_out.saveImageSequence(outpath, outtype, plane);
}

void paintPhaseImage( const char *outpath,
					  dmod::array2d &input,
					  EKFState &tunedState )
{
	size_t N = input.size();
	dmod::array2d output(N);
	for (int i = 0; i < N; ++i)
	{
		EKF filter(tunedState);
		std::vector<Eigen::Vector4d> states = filter.estimateAll(input[i]);
		output[i] = dmod::get_parameter_vector(states, FREQUENCY);
		dmod::normalize(output[i], 0, 255); 
		
		dmod::array1d restoredSignal = filter.getRestoredSignal(states);
		printer::print_states("MatlabScripts/EKF_real_data.txt", states);
		printer::print_signal("MatlabScripts/signal.txt", input[i]);
		printer::print_signal("MatlabScripts/restoredSignal.txt", restoredSignal);
	}

	cv::Mat mat = dmod::matFromArray2d(output);
	cv::imwrite(outpath, mat) ;

}

/////////////////////
///Tuning///////////
/////////////////////
EKFState getTunedKalmanState_TotalSearch( std::vector<dmod::array1d> signals, 
										  size_t filtersAmount, 
										  std::default_random_engine &gen )
{
	//Creation of EKF tuned by TotalSearch
	EKFState minimal;
	EKFState maximal;

	minimal.state = Eigen::Vector4d(0, 0, 0.00, 0);
	minimal.Rw <<
		0.01, 0, 0, 0,
		0, 0.01, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.002;
	minimal.R = minimal.Rw;
	minimal.Rn = 0.1;

	maximal.state = Eigen::Vector4d(255, 10, 0.6, 2 * M_PI);
	maximal.Rw <<
		0.5, 0, 0, 0,
		0, 0.5, 0, 0,
		0, 0, 0.1, 0,
		0, 0, 0, 0.2;
	maximal.R = maximal.Rw;
	maximal.Rn = 10;

	//minimal.state = Eigen::Vector4d(0, 0, 0.00, 0);
	//minimal.Rw <<
	//	0.2, 0, 0, 0,
	//	0, 0.05, 0, 0,
	//	0, 0, 0.001, 0,
	//	0, 0, 0, 0.002;
	//minimal.R = minimal.Rw;
	//minimal.Rn = 0.1;

	//maximal.state = Eigen::Vector4d(255, 5, 0.3, 2 * M_PI);
	//maximal.Rw = minimal.Rw;
	//maximal.R = maximal.Rw;
	//maximal.Rn = 10;

	FilterTuning::TotalSearchTuner tuner(signals, filtersAmount, gen, minimal, maximal);
	tuner.createStates();
	EKFState tunedParameters = tuner.tune();
	
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

/////////////////////
///Scenario///////////
/////////////////////
void sceanrioRealDataOCT_TotalSearch_EveryBScan( const char *path,
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

	paintBScans(outpath, outtype, tomo, tunedState, dmod::Plane::XZ);

	//Need to remember tuned  state! 
	std::stringstream str;
	str << outpath << "tunedState.txt";
	printer::print_full_Kalman_states(str.str().c_str(), std::vector<EKFState>(1, tunedState));
}

void sceanrioRealDataOCT_TotalSearch_Gradient(  const char *path,
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
		0, 0, 0, 0.01;
	step.R = step.Rw;
	step.Rn = 0.1;

	tunedState = getTunedKalmanState_Gradient(tunedState, step, learningData, 200);

	paintBScans(outpath, outtype, tomo, tunedState, dmod::Plane::XZ);

	//Need to remember tuned  state! 
	std::stringstream str;
	str << outpath << "tunedState.txt";
	printer::print_full_Kalman_states(str.str().c_str(), std::vector<EKFState>(1, tunedState));
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

void scenarioRealDataOCT_KalmanProcessing( const char *path,
										   const char *type,
										   size_t begin_number,
										   size_t end_number,
										   EKFState &state,
										   const char *outpath,
								  		   const char *outtype, 
										   dmod::Plane plane )
{
	dmod::Tomogram tomo;
	tomo.initSizeFromImageSequence(path, type, begin_number, end_number);
	tomo.loadImageSequence(path, type, begin_number, end_number);
	paintBScans(outpath, outtype, tomo, state, plane);
}

void scenarioRealDataPhase_TotalSearch( const char *path,
										size_t signalsAmount,
										size_t filtersAmount,
										std::default_random_engine &gen,
										const char *outpath )
{
	cv::Mat mat = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
	dmod::array2d input = dmod::array2dFromMat(mat);
	std::vector<dmod::array1d> learningData = getLearningSignals(signalsAmount, input, gen);
	EKFState tunedState = getTunedKalmanState_TotalSearch(learningData, filtersAmount, gen);
	paintPhaseImage(outpath, input, tunedState);
}

void scenarioCovMatAnalysis(const char *path, dmod::array1d &signal, EKFState &state)
{
	EKF filter(state);
	std::vector<EKFState> fullStates = filter.estimateAllFullStates(signal);
	printer::print_cov_matrices(path, fullStates);
	printer::print_full_Kalman_states("MatlabScripts/full_states.txt", fullStates);
	//for (auto iter = fullStates.begin(); iter != fullStates.end(); ++iter)
	//{
	//	printer::console_print_full_Kalman_state(*iter);
	//}

	filter = EKF(state);
	std::vector<Eigen::Vector4d> states = filter.estimateAll(signal);
	dmod::array1d restoredSignal = filter.getRestoredSignal(states);
	printer::print_states("MatlabScripts/EKF_real_data.txt", states);
	printer::print_signal("MatlabScripts/signal.txt", signal);
	printer::print_signal("MatlabScripts/restoredSignal.txt", restoredSignal);
}

dmod::array1d makeSignal(size_t N, float delta_z, std::default_random_engine &gen)
{
	dmod::array1d background(N, 0);
	dmod::array1d amplitude = dmod::sum(dmod::fixedGaussianAmplitude(0.8, 30, 250, N, 1), dmod::array1d(N, 0.2));
	dmod::array1d frequency(N, 0.1);

	//for (int i = 1; i < N; ++i)
	//{
	//	frequency[i] = frequency[0] - 0.0006*i;
	//}

	dmod::array1d phase = dmod::phaseFromFrequency(frequency, 0, delta_z);
	dmod::array1d noise = dmod::createNormalNoise(0, 0.1, N, gen);
	//dmod::array1d noise(N, 0);
	dmod::array1d signal = dmod::createSignal1D(background, amplitude, phase, noise); 
	return std::move(signal);
}


int main( int argc, char **argv ) 
{
	std::default_random_engine gen((unsigned int)time(NULL));
	dmod::array1d signal = makeSignal(500, 1, gen);

	//dmod::Tomogram tomo;
	//tomo.initSizeFromImageSequence("D:/Data/ZhukovaSignals/egg1_resized/img", ".bmp", 1, 500);
	//tomo.loadImageSequence("D:/Data/ZhukovaSignals/egg1_resized/img", ".bmp", 1, 500);

	//std::vector<dmod::array1d> learningData;
	//learningData = getLearningSignals(150, tomo, gen);

	//EKFState state = getTunedKalmanState_TotalSearch(learningData, 10000, gen);



	//dmod::array1d signal = tomo.getSignal1D(200, 150, 0, dmod::Axis::Z);

	EKFState state;
	state.state = Eigen::Vector4d(0, 0.2, 0.1, 0);
	state.Rw <<
		0.09, 0, 0, 0,
		0, 0.05, 0, 0,
		0, 0, 0.001, 0,
		0, 0, 0, 0.02;
	state.R = state.Rw;
	state.Rn = 0.4;



	scenarioCovMatAnalysis("MatlabScripts/cov_mats.txt", signal, state);

	//sceanrioRealDataOCT_TotalSearch( "D:/Data/ZhukovaSignals/onion1_resized/img", ".bmp", 1, 500,
	//								 "D:/Data/ZhukovaSignals/onion1_resized/mask.bmp",
	//								 150, 1000, gen,
	//								 "D:/Data/ZhukovaSignals/onion1_resized/output_3/", ".bmp" );

	//sceanrioRealDataOCT_TotalSearch( "D:/Data/ZhukovaSignals/onion2_resized/img", ".bmp", 1, 500,
	//								 "D:/Data/ZhukovaSignals/onion2_resized/mask.bmp",
	//								 150, 1000, gen,
	//								 "D:/Data/ZhukovaSignals/onion2_resized/output_3/", ".bmp" );


	//sceanrioRealDataOCT_TotalSearch( "D:/Data/ZhukovaSignals/egg1_resized/img", ".bmp", 1, 500,
	//								 "D:/Data/ZhukovaSignals/egg1_resized/mask.bmp",
	//							     150, 1000, gen,
	//								 "D:/Data/ZhukovaSignals/egg1_resized/output_3/", ".bmp" );



	//const char *path = "D:/Data/ZhukovaSignals/onion1_resized/img";
	//const char *outpath = "D:/Data/ZhukovaSignals/onion1_resized/output_2/";

	//dmod::Tomogram tomo;
	//tomo.initSizeFromImageSequence(path, ".bmp", 1, 500);
	//tomo.loadImageSequence(path, ".bmp", 1, 500);

	//EKFState state;
	//state.state = Eigen::Vector4d(1, 0, 0.01, 0.1);
	//state.Rw <<
	//	0.01, 0, 0, 0,
	//	0, 0.01, 0, 0,
	//	0, 0, 0.001, 0,
	//	0, 0, 0, 0.01;
	//state.R = state.Rw;
	//state.Rn = 0.1;

	//paintBScans(outpath, ".bmp", tomo, state, dmod::Plane::XZ);

	//dmod::array1d signal = makeSignal(500, 1, gen);
	//scenarioCovMatAnalysis("MatlabScripts/cov_mats.txt", signal, state) ;

	//scenarioRealDataPhase_TotalSearch( "D:/Data/ZhukovaSignals/phases_17/pic_r60_10_840_gn30.bmp", 
	//								   30, 1000, gen,
	//								   "D:/Data/ZhukovaSignals/phases_17/my_out.bmp" );

	return 0;
}


//dmod::array1d restoredSignal = filter.getRestoredSignal(states);
//printer::print_states("MatlabScripts/EKF_real_data.txt", states);
//printer::print_signal("MatlabScripts/signal.txt", signal);
//printer::print_signal("MatlabScripts/restoredSignal.txt", restoredSignal);


//const char *path = "D:/Data/ZhukovaSignals/onion1_resized/img";
//const char *outpath = "D:/Data/ZhukovaSignals/onion1_resized/output_2/";
//
//dmod::Tomogram tomo;
//tomo.initSizeFromImageSequence(path, ".bmp", 1, 500);
//tomo.loadImageSequence(path, ".bmp", 1, 500);

//EKFState state;
//state.state = Eigen::Vector4d(1, 0, 0.01, 0.1);
//state.Rw <<
//	0.01, 0, 0, 0,
//	0, 0.01, 0, 0,
//	0, 0, 0.001, 0,
//	0, 0, 0, 0.01;
//state.R = state.Rw;
//state.Rn = 0.1;

//paintBScans(outpath, ".bmp", tomo, state, dmod::Plane::XZ);
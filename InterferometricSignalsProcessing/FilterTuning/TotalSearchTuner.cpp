#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "TotalSearchTuner.h"

FilterTuning::TotalSearchTuner::TotalSearchTuner(float **inputSignals_, int signalSize_, int signalsCount_, int filtersCount_,
	std::default_random_engine &gen_, ExtendedKalmanFilterIS1DState min_, ExtendedKalmanFilterIS1DState max_)
	: inputSignals(inputSignals_), signalSize(signalSize_), signalsCount(signalsCount_), 
	filtersCount(filtersCount_), gen(gen_), minimal(min_), maximal(max_), filterStates (NULL) {}

FilterTuning::TotalSearchTuner::~TotalSearchTuner()
{
	if (filterStates != NULL)
	{
		delete[] filterStates;
	}
}

ExtendedKalmanFilterIS1DState FilterTuning::TotalSearchTuner::createRandomState()
{
	ExtendedKalmanFilterIS1DState randomState;
	for (int i = 0; i < 4; i++)		//state
	{
		randomState.state(i) = getRandom(minimal.state(i), maximal.state(i));
	}
	for (int i = 0; i < 4; i++)		//Rw and R
	{
		for (int j = 0; j < 4; j++)
		{
			randomState.Rw(i, j) = getRandom(minimal.Rw(i, j), maximal.Rw(i, j));
			randomState.R(i, j) = getRandom(minimal.R(i, j), maximal.R(i, j));
		}
	}
	randomState.Rn = getRandom(minimal.Rn, maximal.Rn) ;	//Rn
	return randomState;
}
void FilterTuning::TotalSearchTuner::createStates()
{
	if (filterStates == NULL)
	{
		filterStates = new ExtendedKalmanFilterIS1DState[filtersCount];
	}
	for (int i = 0; i < filtersCount; i++)
	{
		filterStates[i] = createRandomState();
		std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((float)i + 1)/filtersCount * 100 << "%";
	}
	std::cout << std::endl;
}

void FilterTuning::TotalSearchTuner::changeSignals(float **inputSignals_, int signalsCount_)
{
	inputSignals = inputSignals_;
	signalsCount = signalsCount_;
}

float FilterTuning::TotalSearchTuner::getRandom(float min, float max)
{
	if (abs(max - min) < 0.000000001)		// if max == min (max - min == 0)
		return min;
	return ((float)(gen() % 100000000)) / float(100000000) * (max - min) + min;
}

//TODO rewrite simpler!!!
ExtendedKalmanFilterIS1DState FilterTuning::TotalSearchTuner::tune() 
{
	return ExtendedKalmanFilterIS1DState();

	////Creation of filters
	//if (filterStates == NULL)
	//{
	//	createStates();
	//}
	//ExtendedKalmanFilterIS1D *filters = new ExtendedKalmanFilterIS1D[filtersCount];
	//float ***recSignals = new float**[signalsCount] ;
	//for (int i = 0; i < signalsCount; i++)
	//{
	//	recSignals[i] = new float*[filtersCount];
	//}
	//for (int i = 0; i < signalsCount; i++)
	//{
	//	for (int j = 0; j < filtersCount; j++)
	//	{
	//		recSignals[i][j] = new float[signalSize];
	//	}
	//}
	////Estimation
	//for (int i = 0; i < signalsCount; i++)
	//{
	//	for (int j = 0; j < filtersCount; j++)
	//	{
	//		filters[j] = ExtendedKalmanFilterIS1D(filterStates[j]);
	//		for (int k = 0; k < signalSize; k++)
	//		{
	//			filters[j].estimate(inputSignals[i][k]);
	//			recSignals[i][j][k] = filters[j].evaluateSignalValue();
	//		}
	//		//StatePrinter::console_print_full_Kalman_state(filterStates[j]);
	//		//StatePrinter::print_signal("out.txt", recSignals[i][j], signalSize) ;
	//		std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((float)i*filtersCount + j + 1) / (signalsCount*filtersCount) * 100 << "%";
	//	}
	//}
	//std::cout << std::endl;

	////Best filter search
	//float *vars = new float[filtersCount];
	//float *difference = new float[signalSize] ;
	//for (int j = 0; j < filtersCount; j++)
	//{
	//	vars[j] = 0;
	//	for (int i = 0; i < signalsCount; i++)
	//	{
	//		dmod::sub(inputSignals[i], recSignals[i][j], difference, signalSize);
	//		vars[j] += dmod::var(difference, signalSize);
	//		std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((float)j*signalsCount + i + 1) / (signalsCount*filtersCount) * 100 << "%";
	//	}
	//}
	//std::cout << std::endl;

	//int best_index = dmod::min_index(vars, filtersCount);

	////Memoryy release
	//delete[] vars;
	//delete[] difference;
	//for (int i = 0; i < signalsCount; i++)
	//{
	//	for (int j = 0; j < filtersCount; j++)
	//	{
	//		delete[] recSignals[i][j] ;
	//	}
	//}
	//for (int i = 0; i < signalsCount; i++)
	//{
	//	delete[] recSignals[i];
	//}
	//delete[] recSignals;
	//delete[] filters; 

	//return filterStates[best_index];
}


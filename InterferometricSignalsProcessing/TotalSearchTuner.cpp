#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "SignalAnalysis.h"
#include "StatePrinter.h"
#include "TotalSearchTuner.h"


FilterTuning::TotalSearchTuner::TotalSearchTuner(double **inputSignals_, int signalSize_, int signalsCount_, int filtersCount_,
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
	for (int i = 0; i < 4; i++)		//Rw
	{
		for (int j = 0; j < 4; j++)
		{
			randomState.Rw(i, j) = getRandom(minimal.Rw(i, j), maximal.Rw(i, j));
		}
	}
	for (int i = 0; i < 4; i++)		//R
	{
		for (int j = 0; j < 4; j++)
		{
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
		std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((double)i + 1)/filtersCount * 100 << "%";
	}
	std::cout << std::endl;
}

void FilterTuning::TotalSearchTuner::changeSignals(double **inputSignals_, int signalsCount_)
{
	inputSignals = inputSignals_;
	signalsCount = signalsCount_;
}

double FilterTuning::TotalSearchTuner::getRandom(double min, double max)
{
	if (abs(max - min) < 0.000000001)		// if max == min (max - min == 0)
		return min;
	return ((double)(gen() % 100000000)) / double(100000000) * (max - min) + min;
}

ExtendedKalmanFilterIS1DState FilterTuning::TotalSearchTuner::tune()
{
	//Creation of filters
	if (filterStates == NULL)
	{
		createStates();
	}
	ExtendedKalmanFilterIS1D *filters = new ExtendedKalmanFilterIS1D[filtersCount];
	double ***recSignals = new double**[signalsCount] ;
	for (int i = 0; i < signalsCount; i++)
	{
		recSignals[i] = new double*[filtersCount];
	}
	for (int i = 0; i < signalsCount; i++)
	{
		for (int j = 0; j < filtersCount; j++)
		{
			recSignals[i][j] = new double[signalSize];
		}
	}
	//Estimation
	for (int i = 0; i < signalsCount; i++)
	{
		for (int j = 0; j < filtersCount; j++)
		{
			filters[j] = ExtendedKalmanFilterIS1D(filterStates[j]);
			for (int k = 0; k < signalSize; k++)
			{
				filters[j].estimate(inputSignals[i][k]);
				recSignals[i][j][k] = filters[j].evaluateSignalValue();
			}
			//StatePrinter::console_print_full_Kalman_state(filterStates[j]);
			//StatePrinter::print_signal("out.txt", recSignals[i][j], signalSize) ;
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((double)i*filtersCount + j + 1) / (signalsCount*filtersCount) * 100 << "%";
		}
	}
	std::cout << std::endl;

	//Best filter search
	double *variations = new double[filtersCount];
	double *difference = new double[signalSize] ;
	for (int j = 0; j < filtersCount; j++)
	{
		variations[j] = 0;
		for (int i = 0; i < signalsCount; i++)
		{
			SignalAnalysis::diff(inputSignals[i], recSignals[i][j], difference, signalSize);
			variations[j] += SignalAnalysis::var(difference, signalSize);
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((double)j*signalsCount + i + 1) / (signalsCount*filtersCount) * 100 << "%";
		}
	}
	std::cout << std::endl;

	int best_index = SignalAnalysis::min_index(variations, filtersCount);

	//Memoryy release
	delete[] variations;
	delete[] difference;
	for (int i = 0; i < signalsCount; i++)
	{
		for (int j = 0; j < filtersCount; j++)
		{
			delete[] recSignals[i][j] ;
		}
	}
	for (int i = 0; i < signalsCount; i++)
	{
		delete[] recSignals[i];
	}
	delete[] recSignals;
	delete[] filters; 

	return filterStates[best_index];
}



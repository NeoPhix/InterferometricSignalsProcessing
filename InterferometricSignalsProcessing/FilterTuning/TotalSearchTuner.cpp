#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "FilterTuning.h"

#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "TotalSearchTuner.h"

using namespace FilterTuning;

FilterTuning::TotalSearchTuner::TotalSearchTuner(std::vector<dmod::array1d> &inputSignals_, int signalSize_, int signalsAmount_, int filtersAmount_,
	std::default_random_engine &gen_, EKFState min_, EKFState max_)
	: inputSignals(inputSignals_), signalSize(signalSize_), signalsAmount(signalsAmount_), 
	filtersAmount(filtersAmount_), gen(gen_), minimal(min_), maximal(max_), filterStates (NULL) {}

FilterTuning::TotalSearchTuner::~TotalSearchTuner()
{
}

EKFState FilterTuning::TotalSearchTuner::createRandomState()
{
	EKFState randomState;
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
	if (filterStates.empty())
	{
		filterStates.reserve(filtersAmount);
	}
	for (int i = 0; i < filtersAmount; ++i)
	{
		filterStates[i] = createRandomState();
		std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((double)i + 1) / filtersAmount * 100 << "%";
	}
}

void FilterTuning::TotalSearchTuner::changeSignals(std::vector<dmod::array1d> &inputSignals_)
{
	inputSignals = std::move(inputSignals_);
	signalsAmount = inputSignals.size();
}

double FilterTuning::TotalSearchTuner::getRandom(double min, double max)
{
	if (abs(max - min) < 0.000000001)		// if max == min (max - min == 0)
		return min;
	return ((double)(gen() % 100000000)) / double(100000000) * (max - min) + min;
}

dmod::array3d FilterTuning::TotalSearchTuner::getReconstructedSignals()
{
	if (filterStates.empty())
	{
		createStates();
	}

	dmod::array3d reconstructedSignals = dmod::createArray3d(signalsAmount, filtersAmount, signalSize);

	//Estimation
	for (int i = 0; i < signalsAmount; ++i)
	{
		for (int j = 0; j < filtersAmount; ++j)
		{
			EKF filter = EKF(filterStates[j]);
			reconstructedSignals[i][j] = filter.getRestoredSignal(inputSignals[i]);
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((double)i*filtersAmount + j + 1) / (signalsAmount*filtersAmount) * 100 << "%";
		}
	}
	std::cout << std::endl;

	return reconstructedSignals;
}

EKFState FilterTuning::TotalSearchTuner::tune()
{	
	dmod::array3d reconstructedSignals = getReconstructedSignals();
	//Best filter search
	dmod::array1d variations(filtersAmount);
	for (int j = 0; j < filtersAmount; ++j)
	{
		variations[j] = 0;
		for (int i = 0; i < signalsAmount; ++i)
		{
			dmod::array1d diff = dmod::sub(inputSignals[i], reconstructedSignals[i][j]);
			variations[j] += dmod::var(diff);
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << ((double)j*signalsAmount + i + 1) / (signalsAmount*filtersAmount) * 100 << "%";
		}
	}
	std::cout << std::endl;

	int best_index = dmod::min_index(variations);
	return filterStates[best_index];
}


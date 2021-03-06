#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "FilterTuning.h"

#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "TotalSearchTuner.h"

namespace FilterTuning
{

	TotalSearchTuner::TotalSearchTuner( std::vector<dmod::array1d> &inputSignals_,
										int filtersAmount_,
										std::default_random_engine &gen_, 
										EKFState min_, 
										EKFState max_ )
		: inputSignals(inputSignals_), 
		  filtersAmount(filtersAmount_),
		  gen(gen_), 
		  minimal(min_), 
		  maximal(max_), 
		  filterStates(NULL) {}

	TotalSearchTuner::~TotalSearchTuner()
	{
	}

	EKFState TotalSearchTuner::createRandomState()
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
		randomState.Rn = getRandom(minimal.Rn, maximal.Rn);	//Rn
		return randomState;
	}

	void TotalSearchTuner::createStates()
	{
		if (filterStates.empty())
		{
			filterStates.resize(filtersAmount);
		}
		for (int i = 0; i < filtersAmount; ++i)
		{
			filterStates[i] = createRandomState();
		}
	}

	void TotalSearchTuner::changeSignals(std::vector<dmod::array1d> &inputSignals_)
	{
		inputSignals = std::move(inputSignals_);
	}

	float TotalSearchTuner::getRandom(float min, float max)
	{
		if (abs(max - min) < 0.000000001)		// if max == min (max - min == 0)
			return min;
		return ((float)(gen() % 100000000)) / float(100000000) * (max - min) + min;
	}

	EKFState TotalSearchTuner::tune()
	{
		if (filterStates.empty())
		{
			createStates();
		}

		dmod::array1d variations(filtersAmount);
		for (int j = 0; j < filtersAmount; ++j)
		{
			variations[j] = fitness(inputSignals, filterStates[j]);
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << (double)(j + 1) / filtersAmount * 100 << "%%";
		}
		std::cout << std::endl;

		int best_index = dmod::min_index(variations);

		return filterStates[best_index];
	}

}
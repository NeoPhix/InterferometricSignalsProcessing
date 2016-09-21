#ifndef TOTAL_SEARCH_TUNER_H
#define TOTAL_SEARCH_TUNER_H

#include <random>
#include <Eigen\Dense>
#include "ExtendedKalmanFilterIS1D.h"

namespace FilterTuning
{
	class TotalSearchTuner
	{
	public:
		TotalSearchTuner(double **inputSignals_, int signalsCount_, int filtersCount_, std::default_random_engine &gen_, ExtendedKalmanFilterIS1DState min_, ExtendedKalmanFilterIS1DState max_);
		~TotalSearchTuner();

		void createStates();
		void changeSignals(double **inputSignals_, int signalsCount_);

		ExtendedKalmanFilterIS1DState tune();
	private:
		ExtendedKalmanFilterIS1DState createRandomState();
		double getRandom(double min, double max);

		double **inputSignals;
		ExtendedKalmanFilterIS1DState *filterStates;
		int signalsCount;
		int filtersCount;
		std::default_random_engine gen ;
		ExtendedKalmanFilterIS1DState minimal;
		ExtendedKalmanFilterIS1DState maximal;
	} ;
}

#endif

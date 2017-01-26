#ifndef TOTAL_SEARCH_TUNER_H
#define TOTAL_SEARCH_TUNER_H

#include <random>

#include "../Filters/ExtendedKalmanFilterIS1D.h"
#include "../DataModel/SignalAnalysis.h"

namespace FilterTuning
{
	class TotalSearchTuner
	{
	public:
		TotalSearchTuner(float **inputSignals_, int signalSize_, int signalsCount_, int filtersCount_,
			std::default_random_engine &gen_, ExtendedKalmanFilterIS1DState min_, ExtendedKalmanFilterIS1DState max_);
		~TotalSearchTuner();

		void createStates();
		void changeSignals(float **inputSignals_, int signalsCount_);

		ExtendedKalmanFilterIS1DState tune();
	private:
		ExtendedKalmanFilterIS1DState createRandomState();
		float getRandom(float min, float max);

		//todo
		float **inputSignals;
		ExtendedKalmanFilterIS1DState *filterStates;
		
		//new
		//std::vector<dmod::signal1d> inputSignals;
		//std::vector<ExtendedKalmanFilterIS1DState> filterStates;
		//
		int signalsCount;
		int signalSize;
		int filtersCount;
		std::default_random_engine gen ;
		ExtendedKalmanFilterIS1DState minimal;
		ExtendedKalmanFilterIS1DState maximal;
	} ;
}

#endif

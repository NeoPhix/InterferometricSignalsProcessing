#ifndef TOTAL_SEARCH_TUNER_H
#define TOTAL_SEARCH_TUNER_H

#include <random>

#include "FilterTuning.h"

#include "../Filters/ExtendedKalmanFilterIS1D.h"
#include "../DataModel/SignalAnalysis.h"

using namespace FilterTuning;

namespace FilterTuning
{
	class TotalSearchTuner
	{
	public:
		TotalSearchTuner(std::vector<dmod::array1d> &inputSignals_, int filtersAmount_,
			std::default_random_engine &gen_, EKFState min_, EKFState max_);
		~TotalSearchTuner();

		void createStates();
		void changeSignals(std::vector<dmod::array1d> &inputSignals_);

		EKFState tune();
	private:
		EKFState createRandomState();
		double getRandom(double min, double max);

	private:
		std::vector<dmod::array1d> inputSignals;
		int filtersAmount;

		std::default_random_engine gen ;

		EKFState minimal;
		EKFState maximal;

		std::vector<EKFState> filterStates;

	} ;
}

#endif

#ifndef FILTER_TUNING_H
#define FILTER_TUNING_H

#include "../Filters/ExtendedKalmanFilterIS1D.h"

#include "../DataModel/SignalAnalysis.h""

namespace FilterTuning
{
	typedef ExtendedKalmanFilterIS1DState EKFState;
	typedef ExtendedKalmanFilterIS1D EKF;

	double fitness(std::vector<dmod::array1d> &inputSignals, EKFState filterState);
}

#endif
#ifndef FILTER_TUNING_H
#define FILTER_TUNING_H

#include "../Filters/ExtendedKalmanFilterIS1D.h"
#include "../DataModel/SignalAnalysis.h""

namespace FilterTuning
{
	double fitness(std::vector<dmod::signal1d> &inputSignals, ExtendedKalmanFilterIS1DState filterState);
}

#endif
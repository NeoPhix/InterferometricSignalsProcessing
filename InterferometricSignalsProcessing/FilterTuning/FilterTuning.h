#ifndef FILTER_TUNING_H
#define FILTER_TUNING_H

#include "../Filters/ExtendedKalmanFilterIS1D.h"

namespace FilterTuning
{
	double fitness(double **inputSignals, int signalsCount, int signalSize, ExtendedKalmanFilterIS1DState filterState);
}

#endif
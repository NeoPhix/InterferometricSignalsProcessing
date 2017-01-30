#include "../Filters/ExtendedKalmanFilterIS1D.h"
#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "FilterTuning.h"

float FilterTuning::fitness(std::vector<dmod::array1d> &inputSignals, EKFState filterState)
{
	float sum = 0;
	for (auto iter = inputSignals.begin(); iter != inputSignals.end(); ++iter)
	{
		EKF filter(filterState);
		dmod::array1d reconstructedSignal = filter.getRestoredSignal(*iter);
		dmod::array1d diff = dmod::sub(*iter, reconstructedSignal);
		sum += dmod::var(diff);
	}
	return sum;		//sum of variations of differences between original and reconstructed by EKF results signals
}
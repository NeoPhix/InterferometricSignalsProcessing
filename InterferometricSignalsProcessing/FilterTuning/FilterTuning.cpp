#include "../Filters/ExtendedKalmanFilterIS1D.h"
#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "FilterTuning.h"

double FilterTuning::fitness(std::vector<dmod::array1d> &inputSignals, EKFState filterState)
{
	size_t signalsCount = inputSignals.size();
	if (signalsCount < 1)
	{
		return -1; //Empty signals array
	}
	size_t signalSize = inputSignals[0].size();
	dmod::array1d reconstructed(signalSize);


	//it can be simpler! 
	double sum = 0;
	for (int i = 0; i < signalsCount; i++)
	{
		EKF filter(filterState);

		for (int k = 0; k < signalSize; k++)
		{
			filter.estimate(inputSignals[i][k]);
			reconstructed[k] = filter.evaluateSignalValue();
		}

		dmod::array1d diff = dmod::sub(inputSignals[i], reconstructed);
		sum += dmod::var(diff);
	}
	return sum;
}
#include "../Filters/ExtendedKalmanFilterIS1D.h"
#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "FilterTuning.h"

double FilterTuning::fitness(std::vector<dmod::array1d> &inputSignals, ExtendedKalmanFilterIS1DState filterState)
{
	int signalsCount = inputSignals.size();
	if (signalsCount < 1)
	{
		return -1; //Empty signals array
	}
	int signalSize = inputSignals[0].size();
	dmod::array1d reconstructed(signalSize);

	double sum = 0;
	for (int i = 0; i < signalsCount; i++)
	{
		ExtendedKalmanFilterIS1D filter(filterState);

		for (int k = 0; k < signalSize; k++)
		{
			filter.estimate(inputSignals[i][k]);
			reconstructed[k] = filter.evaluateSignalValue();
		}

//		dmod::array1d difference = dmod::sub(inputSignals[i], reconstructed);
	//	sum += dmod::var(difference);
	}
	return sum;
}
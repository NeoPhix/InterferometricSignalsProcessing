#include "FilterTuning.h"
#include "ExtendedKalmanFilterIS1D.h"
#include "SignalAnalysis.h"
#include "StatePrinter.h"

double FilterTuning::fitness(double **inputSignals, int signalsCount, int signalSize, ExtendedKalmanFilterIS1DState filterState)
{
	//StatePrinter::console_print_full_Kalman_state(filterState) ;

	double *difference = new double[signalSize];
	double *reconstructed = new double[signalSize];
	double sum = 0;
	for (int i = 0; i < signalsCount; i++)
	{
		ExtendedKalmanFilterIS1D filter(filterState);
		for (int k = 0; k < signalSize; k++)
		{
			filter.estimate(inputSignals[i][k]);
			reconstructed[i] = filter.evaluateSignalValue();
		}
		SignalAnalysis::diff(inputSignals[i], reconstructed, difference, signalSize);
		sum += SignalAnalysis::var(difference, signalSize);
	}
	delete[] difference;
	delete[] reconstructed;
	return sum;
}
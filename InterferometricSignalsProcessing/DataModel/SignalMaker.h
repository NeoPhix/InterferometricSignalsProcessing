#ifndef SignalMaker_H
#define SignalMaker_H

#include "SignalAnalysis.h"

namespace dmod
{
	inline double getSignalValue(double background, double amplitude, double phase);
	inline double getGaussValue(double x, double mean, double sigma);

	array1d createSignal1D(const array1d &background, const array1d &amplitude, const array1d &phase, const array1d &noise);
	array1d createNormalNoise(double mean, double sigma, size_t N, std::default_random_engine &gen);

	array1d phaseFromFrequency(array1d &frequency, double startPhase = 0, double delta_z = 1);
	
	array1d fixedGaussianAmplitude(double max, double sigma, double mean, size_t N, double delta_z = 1);
	array1d randomGaussianAmplitude(double max, double sigma, size_t N, size_t minDistance, int maxAmount, std::default_random_engine &gen, double delta_z = 1);
}
#endif
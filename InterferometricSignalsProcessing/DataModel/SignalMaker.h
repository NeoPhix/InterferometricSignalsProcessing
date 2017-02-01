#ifndef SignalMaker_H
#define SignalMaker_H

#include "SignalAnalysis.h"

namespace dmod
{
	inline float getSignalValue(float background, float amplitude, float phase);
	inline float getGaussValue(float x, float mean, float sigma);

	array1d createSignal1D(const array1d &background, const array1d &amplitude, const array1d &phase, const array1d &noise);
	array1d createNormalNoise(float mean, float sigma, size_t N, std::default_random_engine &gen);
	array1d createUniformNoise(float minimal, float maximal, size_t N, std::default_random_engine &gen);

	array1d phaseFromFrequency(array1d &frequency, float startPhase = 0, float delta_z = 1);
	
	array1d fixedGaussianAmplitude(float max, float sigma, float mean, size_t N, float delta_z = 1);
	array1d randomGaussianAmplitude(float max, float sigma, size_t N, size_t minDistance, int maxAmount, std::default_random_engine &gen, float delta_z = 1);
}
#endif
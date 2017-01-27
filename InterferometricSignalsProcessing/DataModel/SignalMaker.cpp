#include <ctime>
#include <cmath>
#include <random>
#include <iostream>
#include <Eigen/Dense>

#include "SignalMaker.h"

using namespace dmod;

double dmod::getSignalValue(double background, double amplitude, double phase)
{
	return background + amplitude * cos(phase);
}

double dmod::getGaussValue(double x, double mean, double sigma)
{
	return exp(-((x - mean)*(x - mean)) / (sigma*sigma));
}

array1d dmod::createSignal1D(const array1d &background, const array1d &amplitude, const array1d &phase, const array1d &noise)
{
	size_t N = background.size();
	if (N != amplitude.size() || N != phase.size() || N != noise.size())
	{
		std::cout << "Arrays sizes are mismatched!" << std::endl;
		return array1d();
	}

	array1d signal(N);
	for (int i = 1; i < N; ++i)
	{
		signal[i] = getSignalValue(background[i], amplitude[i], phase[i]) + noise[i];
	}
	return signal;
}

array1d dmod::createNormalNoise(double mean, double sigma, size_t N, std::default_random_engine &gen)
{
	array1d noise(N);
	std::normal_distribution<double> distribution(mean, sigma);
	for (int i = 0; i < N; ++i)
	{
		noise[i] = distribution(gen);
	}
	return noise;
}

array1d dmod::phaseFromFrequency(array1d &frequency, double startPhase, double delta_z)
{
	size_t N = frequency.size();
	array1d phase(N);
	phase[0] = startPhase;
	for (int i = 1; i < N; ++i)
	{
		phase[i] = phase[i - 1] + 2 * M_PI * frequency[i] * delta_z;
	}
	return phase;
}


array1d dmod::fixedGaussianAmplitude(double max, double sigma, double mean, size_t N, double delta_z)
{
	array1d amplitude(N);

	for (int i = 0; i < N; ++i)
	{
		amplitude[i] = max * getGaussValue(i * delta_z, mean, sigma);
	}

	return amplitude;
}

array1d dmod::randomGaussianAmplitude(double max, double sigma, size_t N, size_t minDistance, int maxAmount, std::default_random_engine &gen, double delta_z)
{
	array1d amplitude(N);
	int amount = gen() % maxAmount + 1;		//Count of gaussian amplitudes in out signal
	
	for (int i = 0; i < amount; ++i)
	{
		size_t pos = (gen() % (N / minDistance)) * minDistance;	//Minimal distance between two random gaussian functions must be kept
		double mean = pos*delta_z;
		amplitude = sum(amplitude, fixedGaussianAmplitude(max, sigma, mean, N, delta_z));
	}

	return amplitude;
}
//

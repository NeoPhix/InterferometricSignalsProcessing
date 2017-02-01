#include <ctime>
#include <cmath>
#include <random>
#include <iostream>
#include <Eigen/Dense>

#include "SignalMaker.h"

namespace dmod
{

	float getSignalValue(float background, float amplitude, float phase)
	{
		return background + amplitude * cos(phase);
	}

	float getGaussValue(float x, float mean, float sigma)
	{
		return exp(-((x - mean)*(x - mean)) / (sigma*sigma));
	}

	array1d createSignal1D(const array1d &background, const array1d &amplitude, const array1d &phase, const array1d &noise)
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
		return std::move(signal);
	}

	array1d createNormalNoise(float mean, float sigma, size_t N, std::default_random_engine &gen)
	{
		array1d noise(N);
		std::normal_distribution<float> distribution(mean, sigma);
		for (int i = 0; i < N; ++i)
		{
			noise[i] = distribution(gen);
		}
		return std::move(noise);
	}

	array1d createUniformNoise(float minimal, float maximal, size_t N, std::default_random_engine &gen)
	{
		array1d noise(N);
		std::uniform_real_distribution<float> distribution(minimal, maximal);
		for (int i = 0; i < N; ++i)
		{
			noise[i] = distribution(gen);
		}
		return std::move(noise);
	}

	array1d phaseFromFrequency(array1d &frequency, float startPhase, float delta_z)
	{
		size_t N = frequency.size();
		array1d phase(N);
		phase[0] = startPhase;
		for (int i = 1; i < N; ++i)
		{
			phase[i] = phase[i - 1] + 2 * M_PI * frequency[i] * delta_z;
		}
		return std::move(phase);
	}


	array1d fixedGaussianAmplitude(float max, float sigma, float mean, size_t N, float delta_z)
	{
		array1d amplitude(N);

		for (int i = 0; i < N; ++i)
		{
			amplitude[i] = max * getGaussValue(i * delta_z, mean, sigma);
		}

		return std::move(amplitude);
	}

	array1d randomGaussianAmplitude(float max, float sigma, size_t N, size_t minDistance, int maxAmount, std::default_random_engine &gen, float delta_z)
	{
		array1d amplitude(N);
		int amount = gen() % maxAmount + 1;		//Count of gaussian amplitudes in out signal

		for (int i = 0; i < amount; ++i)
		{
			size_t pos = (gen() % (N / minDistance)) * minDistance;	//Minimal distance between two random gaussian functions must be kept
			float mean = pos*delta_z;
			amplitude = sum(amplitude, fixedGaussianAmplitude(max, sigma, mean, N, delta_z));
		}

		return std::move(amplitude);
	}
	
}
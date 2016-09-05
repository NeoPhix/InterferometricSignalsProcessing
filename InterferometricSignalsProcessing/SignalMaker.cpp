#include <ctime>
#include <cmath>
#include <random>
#include <Eigen/Dense>

#include "SignalMaker.h"

double* SignalMaker::createSignal1D(double *background, double *amplitude, double *phase, double *noise, int N, double delta_z)
{
	double *signal = new double[N];
	signal[0] = background[0] + amplitude[0] * cos(phase[0]) + noise[0];
	for (int i = 1; i < N; i++)
	{
		signal[i] = background[i] + amplitude[i] * cos(phase[i]) + noise[i];
	}
	return signal;
}

double* SignalMaker::normalDistribution(double mean, double sigma, int N)
{
	double *noise = new double[N];
	std::default_random_engine gen((unsigned int)time(NULL));
	std::normal_distribution<double> distribution(mean, sigma);
	for (int i = 0; i < N; i++)
	{
		noise[i] = distribution(gen);
	}
	return noise;
}

double* SignalMaker::phaseFromFrequency(const double *frequency, int N, double delta_z)
{
	double *phase = new double[N];
	for (int i = 1; i < N; i++)
	{
		phase[i] = phase[i - 1] + 2 * M_PI*frequency[i] * delta_z;
	}
	return phase;
}

double* SignalMaker::phaseFromFrequency(double frequency, int N, double delta_z)
{
	double *phase = new double[N];
	for (int i = 1; i < N; i++)
	{
		phase[i] = phase[i - 1] + 2 * M_PI*frequency * delta_z;
	}
	return phase;
}

double SignalMaker::getSignalValue(double background, double amplitude, double phase)
{
	return background + amplitude * cos(phase);
}

double SignalMaker::gaussianAmplitude(double x, double mean, double sigma)
{
	return exp(-((x - mean)*(x - mean)) / (sigma*sigma));
}

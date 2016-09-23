#ifndef SignalMaker_H
#define SignalMaker_H

namespace SignalMaker
{
	double* createSignal1D(double *background, double *amplitude, double *phase, double *noise, const int N);
	double* normalDistribution(double mean, double sigma, const int N, std::default_random_engine &gen);
	double gaussianAmplitude(double x, double mean, double sigma);
	double* phaseFromFrequency(const double *frequency, double startPhase, int N, double delta_z);
	double* phaseFromFrequency(double frequency, double startPhase, int N, double delta_z);
	double getSignalValue(double background, double amplitude, double phase);
	double* fixedGaussianAmplitude(const int N, double minValue, double maxValue, double sigma, int *edges, int gaussiansCount);
	double* randomGaussianAmplitude(const int N, double minValue, double maxValue, double sigma, int maxGaussiansCount, std::default_random_engine &gen) ;
}

#endif
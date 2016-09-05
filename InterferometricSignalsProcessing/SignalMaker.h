#ifndef SignalMaker_H
#define SignalMaker_H

namespace SignalMaker
{
	double* createSignal1D(double *background, double *amplitude, double *phase, double *noise, int N, double delta_z);
	double* normalDistribution(double mean, double sigma, int N);
	double gaussianAmplitude(double x, double mean, double sigma);
	double* phaseFromFrequency(const double *frequency, int N, double delta_z);
	double* phaseFromFrequency(double frequency, int N, double delta_z);
	double getSignalValue(double background, double amplitude, double phase);
}

#endif
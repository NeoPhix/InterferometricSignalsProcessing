#ifndef SignalMaker_H
#define SignalMaker_H

class SignalMaker
{
public:
	static double* createSignal1D(double *background, double *amplitude, double *phase, double *noise, int N, double delta_z);
	static double* normalDistribution(double mean, double sigma, int N);
	static double gaussianAmplitude(double x, double mean, double sigma);
	static double* phaseFromFrequency(const double *frequency, int N, double delta_z);
	static double* phaseFromFrequency(double frequency, int N, double delta_z);
	static double getSignalValue(double background, double amplitude, double phase);
};

#endif
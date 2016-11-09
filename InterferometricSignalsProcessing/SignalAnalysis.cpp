#include <cmath>
#include <Eigen\Dense>

#include "SignalAnalysis.h"


double SignalAnalysis::mean(const double *s, int N)
{
	double res = 0;
	for (int i = 0; i < N; i++)
	{
		res += s[i];
	}
	res /= N;
	return res;
}

double SignalAnalysis::stdev(const double *s, int N)
{
	return sqrt(var(s, N)) ;
}

double SignalAnalysis::var(const double *s, int N)
{
	double res = 0;
	double mn2 = mean(s, N);
	mn2 *= mn2 ;
	for (int i = 0; i < N; i++)
	{
		res += s[i] * s[i] - mn2 ;
	}
	res /= N - 1;
	return res;
}

double SignalAnalysis::max(const double *s, int N)
{
	double res = s[0];
	for (int i = 1; i < N; i++)
	{
		if (s[i] > res)
		{
			res = s[i];
		}
	}
	return res ;
}

double SignalAnalysis::min(const double *s, int N)
{
	double res = s[0];
	for (int i = 1; i < N; i++)
	{
		if (s[i] < res)
		{
			res = s[i];
		}
	}
	return res;
}

int SignalAnalysis::max_index(const double *s, int N)
{
	double res = s[0];
	int index = 0 ;
	for (int i = 1; i < N; i++)
	{
		if (s[i] > res)
		{
			res = s[i];
			index = i;
		}
	}
	return index;
}

int SignalAnalysis::min_index(const double *s, int N)
{
	double res = s[0];
	int index = 0;
	for (int i = 1; i < N; i++)
	{
		if (s[i] < res)
		{
			res = s[i];
			index = i;
		}
	}
	return index;
}

double SignalAnalysis::snr(const double *s, const double *noise, int N)	//Signal-to-noise ratio
{
	return sqrt(10 * log10(SignalAnalysis::var(s, N) / SignalAnalysis::var(noise, N)));
}

void SignalAnalysis::diff(const double *s1, const double *s2, double *target, int N)
{
	for (int i = 0; i < N; i++)
	{
		target[i] = s1[i] - s2[i] ;
	}
}

Eigen::Vector4d SignalAnalysis::get_deviations(Eigen::Vector4d *states, double *signal, double *noise,
	double *background, double *amplitude, double *frequency, double *phase, double *restoredSignal, int N)
{
	double *tmp = new double[N];
	Eigen::Vector4d result;

	for (int i = 0; i < N; i++)
	{
		tmp[i] = states[i](0) - background[i];
	}
	result(0) = SignalAnalysis::stdev(tmp, N);
	for (int i = 0; i < N; i++)
	{
		tmp[i] = states[i](1) - amplitude[i];
	}
	result(1) = SignalAnalysis::stdev(tmp, N);
	for (int i = 0; i < N; i++)
	{
		tmp[i] = states[i](2) - frequency[i];
	}
	result(2) = SignalAnalysis::stdev(tmp, N);
	for (int i = 0; i < N; i++)
	{
		tmp[i] = states[i](3) - phase[i];
	}
	result(3) = SignalAnalysis::stdev(tmp, N);
	//SignalAnalysis::diff(signal, restoredSignal, tmp, N);
	//result(0) = SignalAnalysis::stdev(tmp, N);

	delete[] tmp;
	return result;
}


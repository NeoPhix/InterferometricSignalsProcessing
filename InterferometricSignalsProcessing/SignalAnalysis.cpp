#include <cmath>

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
	double res = -10000000000;
	for (int i = 0; i < N; i++)
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
	double res = 10000000000;
	for (int i = 0; i < N; i++)
	{
		if (s[i] < res)
		{
			res = s[i];
		}
	}
	return res;
}
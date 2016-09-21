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

void SignalAnalysis::diff(const double *s1, const double *s2, double *target, int N)
{
	for (int i = 0; i < N; i++)
	{
		target[i] = s1[i] - s2[i] ;
	}
}
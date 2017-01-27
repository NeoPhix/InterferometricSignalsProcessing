#include <cmath>
#include <Eigen\Dense>

#include "SignalAnalysis.h"

using namespace dmod;

array2d dmod::createArray2d(size_t h, size_t w)
{
	return array2d(h, array1d(w));
}

array3d dmod::createArray3d(size_t d, size_t h, size_t w)
{
	return array3d(d, createArray2d(h, w));
}

double dmod::mean(const array1d &s)
{
	size_t N = s.size();
	double res = 0;
	for (auto i = 0; i < N; i++)
	{
		res += s[i];
	}
	res /= N;
	return res;
}

double dmod::stdev(const array1d &s)
{
	return sqrt(var(s)) ;
}

double dmod::var(const array1d &s)
{
	size_t N = s.size();
	double res = 0;
	double mn2 = mean(s);
	mn2 *= mn2 ;
	for (int i = 0; i < N; i++)
	{
		res += s[i] * s[i] - mn2 ;
	}
	res /= N - 1;
	return res;
}

double dmod::snr(const array1d &s, const array1d &noise)	//Signal-to-noise ratio
{
	return sqrt(10 * log10(var(s) / var(noise)));
}

double dmod::max(const array1d &s)
{
	size_t N = s.size();
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

double dmod::min(const array1d &s)
{
	size_t N = s.size();
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

int dmod::max_index(const array1d &s)
{
	size_t N = s.size();
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

int dmod::min_index(const array1d &s)
{
	size_t N = s.size();
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

array1d dmod::sub(const array1d &s1, const array1d &s2)
{
	if (s1.size() != s2.size())
	{
		return array1d(0);	//Invalid size
	}
	size_t N = s1.size();
	array1d target(N);

	for (int i = 0; i < N; i++)
	{
		target[i] = s1[i] - s2[i] ;
	}

	return target;
}

array1d dmod::sum(const array1d &s1, const array1d &s2)
{
	if (s1.size() != s2.size())
	{
		return array1d(0);	//Invalid size
	}
	size_t N = s1.size();
	array1d target(N);

	for (int i = 0; i < N; i++)
	{
		target[i] = s1[i] + s2[i];
	}

	return target;
}

//Eigen::signal4d get_deviations(Eigen::signal4d *states, double *signal, double *noise,
//	double *background, double *amplitude, double *frequency, double *phase, double *restoredSignal, int N)
//{
//	double *tmp = new double[N];
//	Eigen::signal4d result;
//
//	for (int i = 0; i < N; i++)
//	{
//		tmp[i] = states[i](0) - background[i];
//	}
//	result(0) = stdev(tmp, N);
//	for (int i = 0; i < N; i++)
//	{
//		tmp[i] = states[i](1) - amplitude[i];
//	}
//	result(1) = stdev(tmp, N);
//	for (int i = 0; i < N; i++)
//	{
//		tmp[i] = states[i](2) - frequency[i];
//	}
//	result(2) = stdev(tmp, N);
//	for (int i = 0; i < N; i++)
//	{
//		tmp[i] = states[i](3) - phase[i];
//	}
//	result(3) = stdev(tmp, N);
//	//diff(signal, restoredSignal, tmp, N);
//	//result(0) = stdev(tmp, N);
//
//	delete[] tmp;
//	return result;
//}


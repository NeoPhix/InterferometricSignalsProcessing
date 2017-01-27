#include <iostream>
#include <cmath>
#include <Eigen/Dense>

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

bool dmod::isEmpty(array1d &s)
{
	return s.empty();
}

bool dmod::isEmpty(array2d &s)
{
	if (!s.empty())
	{
		return isEmpty(s[0]);
	}
	return true;
}

bool dmod::isEmpty(array3d &s)
{
	if (!s.empty())
	{
		return isEmpty(s[0]);
	}
	return true;
}

cv::Mat dmod::matFromArray2d(array2d &input)
{
	if (isEmpty(input))
	{
		std::cout << "Array is empty!" << std::endl;
		return cv::Mat();
	}

	size_t height = input.size();
	size_t width = input[0].size();
	cv::Mat mat(cv::Size(width, height), CV_8UC1);

	uchar *ptr = mat.ptr();
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			*ptr++ = input[y][x];
		}
	}

	return mat;
}

array2d dmod::array2dFromMat(cv::Mat &mat)
{
	if (mat.rows == 0 || mat.cols == 0)
	{
		std::cout << "Mat is empty!" << std::endl;
		return createArray2d(0, 0);
	}
	int height = mat.rows;
	int width = mat.cols;

	array2d res = createArray2d(height, width);

	uchar *ptr = mat.ptr();
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			res[y][x] = *ptr++;
		}
	}

	return res;
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

Eigen::Vector4d dmod::get_deviations(std::vector<Eigen::Vector4d> &states, array1d &background, array1d &amplitude, array1d &frequency, array1d &phase)
{
	int N = states.size();
	if (background.size() != N || amplitude.size() != N || frequency.size() != N || phase.size() != N)
	{
		std::cout << "Error. Arrays sizes are mismatched." << std::endl;
		return Eigen::Vector4d();
	}

	array2d tmp = createArray2d(4, N);
	for (int i = 0; i < N; ++i)
	{
		tmp[0][i] = states[i](0) - background[i];
		tmp[1][i] = states[i](1) - amplitude[i];
		tmp[2][i] = states[i](2) - frequency[i];
		tmp[3][i] = states[i](3) - phase[i];
	}

	Eigen::Vector4d res;
	for (int i = 0; i < 4; ++i)
	{
		res(i) = stdev(tmp[i]);
	}

	return res;
}
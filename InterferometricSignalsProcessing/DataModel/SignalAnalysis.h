#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

#include <vector>

#include <opencv/cv.h>
#include <Eigen/Dense>


namespace dmod
{
	typedef std::vector<float> array1d;
	typedef std::vector<array1d> array2d;
	typedef std::vector<array2d> array3d;

	bool isEmpty(array1d &s);
	bool isEmpty(array2d &s);
	bool isEmpty(array3d &s);

	array2d createArray2d(size_t h, size_t w);
	array3d createArray3d(size_t d, size_t h, size_t w);

	cv::Mat matFromArray2d(array2d &input);
	array2d array2dFromMat(cv::Mat &mat);

	float max(const array1d &s);
	float min(const array1d &s);

	int max_index(const array1d &s);
	int min_index(const array1d &s);

	float mean(const array1d &s);
	float stdev(const array1d &s);
	float var(const array1d &s);
	float snr(const array1d &s, const array1d &noise);
	
	array1d sub(const array1d &s1, const array1d &s2);
	array1d sum(const array1d &s1, const array1d &s2);
	
	Eigen::Vector4d get_deviations(std::vector<Eigen::Vector4d> &states, array1d &background, array1d &amplitude, array1d &frequency, array1d &phase);
}

#endif

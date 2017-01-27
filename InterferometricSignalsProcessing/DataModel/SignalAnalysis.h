#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

#include <Eigen\Dense>
#include <vector>

namespace dmod
{
	typedef std::vector<double> array1d;
	typedef std::vector<array1d> array2d;
	typedef std::vector<array2d> array3d;

	double max(const array1d &s);
	double min(const array1d &s);

	int max_index(const array1d &s);
	int min_index(const array1d &s);

	double mean(const array1d &s);
	double stdev(const array1d &s);
	double var(const array1d &s);
	double snr(const array1d &s, const array1d &noise);
	
	array1d sub(const array1d &s1, const array1d &s2);
	array1d sum(const array1d &s1, const array1d &s2);
	
//	Eigen::Vector4d get_deviations(Eigen::Vector4d *states, double *signal, double *noise,
//		double *background, double *amplitude, double *frequency, double *phase, double *restoredSignal, int N);
}

#endif

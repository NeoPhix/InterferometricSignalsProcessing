#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

namespace SignalAnalysis
{
	double mean(const double *s, int N);
	double stdev(const double *s, int N);
	double var(const double *s, int N);
	double max(const double *s, int N);
	double min(const double *s, int N);
	int max_index(const double *s, int N);
	int min_index(const double *s, int N);
	void diff(const double *s1, const double *s2, double *target, int N);
}

#endif

#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

namespace SignalAnalysis
{
	double stdev(const double *s1, const double *s2, int N);
	double variation(const double *s1, const double *s2, int N);
	double getMaximalValue(const double *s1, int N);
}

#endif

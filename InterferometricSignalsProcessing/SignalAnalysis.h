#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

namespace SignalAnalysis
{
	double mean(const double *s, int N);
	double stdev(const double *s, int N);
	double var(const double *s, int N);
	double max(const double *s, int N);
	double min(const double *s, int N);
}

#endif

#ifndef TOTAL_SEARCH_TUNER_H
#define TOTAL_SEARCH_TUNER_H

#include <random>

namespace FilterTuning
{
	class TotalSearchTuner
	{
	public:
		TotalSearchTuner();
		~TotalSearchTuner();

	private:
		int filtersCount;
		int signalsCount;
	} ;
}

#endif

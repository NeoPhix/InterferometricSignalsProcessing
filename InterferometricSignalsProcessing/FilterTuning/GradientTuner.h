#ifndef GRADIENT_TUNER_H
#define GRADIENT_TUNER_H

#include "../DataModel/SignalAnalysis.h"
#include "../Filters/ExtendedKalmanFilterIS1D.h"

namespace FilterTuning
{
	class GradientTuner
	{
	public:
		GradientTuner( std::vector<dmod::array1d> &inputSignals_, 
					   int iterationsCount_,
					   EKFState currentState_, 
					   EKFState step_ );
		~GradientTuner();

		void changeSignals( std::vector<dmod::array1d> &inputSignals_ );
		EKFState tune();		

	private:
		void makeStep();
		EKFState gradDirection(EKFState &step);
		int gradSign(float s, float interval = 1.);

	private:
		std::vector<dmod::array1d> inputSignals;
		int iterationsCount;
		EKFState currentState;
		EKFState step;

	};
}

#endif

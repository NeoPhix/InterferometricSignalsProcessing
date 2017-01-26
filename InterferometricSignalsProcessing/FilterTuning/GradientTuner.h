#ifndef GRADIENT_TUNER_H
#define GRADIENT_TUNER_H

#include "../Filters/ExtendedKalmanFilterIS1D.h"

namespace FilterTuning
{
	class GradientTuner
	{
	public:
		GradientTuner(float **inputSignals_, int signalSize_, int signalsCount_, int iterationsCount_,
			ExtendedKalmanFilterIS1DState currentState_, ExtendedKalmanFilterIS1DState step_);
		~GradientTuner();

		void changeSignals(float **inputSignals_, int signalsCount_);
		ExtendedKalmanFilterIS1DState tune();
		void makeStep();
	private:
		float **inputSignals;
		int signalSize;
		int signalsCount;
		int iterationsCount;
		ExtendedKalmanFilterIS1DState currentState;
		ExtendedKalmanFilterIS1DState step;

		int gradSign(float s, float interval = 1.);
	};
}

#endif

#ifndef GRADIENT_TUNER_H
#define GRADIENT_TUNER_H

#include "ExtendedKalmanFilterIS1D.h"

namespace FilterTuning
{
	class GradientTuner
	{
	public:
		GradientTuner(double **inputSignals_, int signalSize_, int signalsCount_, int iterationsCount_,
			ExtendedKalmanFilterIS1DState currentState_, ExtendedKalmanFilterIS1DState step_);
		~GradientTuner();

		void changeSignals(double **inputSignals_, int signalsCount_);
		ExtendedKalmanFilterIS1DState tune();
		void makeStep();
	private:
		double **inputSignals;
		int signalSize;
		int signalsCount;
		int iterationsCount;
		ExtendedKalmanFilterIS1DState currentState;
		ExtendedKalmanFilterIS1DState step;

		int gradSign(double s, double interval = 30.);
	};
}

#endif

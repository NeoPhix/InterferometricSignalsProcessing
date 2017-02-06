#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include "../DataModel/SignalAnalysis.h"
#include "../StatePrinter/StatePrinter.h"

#include "FilterTuning.h"
#include "GradientTuner.h"

namespace FilterTuning
{

	GradientTuner::GradientTuner( std::vector<dmod::array1d> &inputSignals_,
								  int iterationsCount_,
								  EKFState currentState_,
								  EKFState step_)
		: inputSignals(inputSignals_),
	      iterationsCount(iterationsCount_),
		  currentState(currentState_),
		  step(step_) {}


	GradientTuner::~GradientTuner() {}

	void FilterTuning::GradientTuner::changeSignals(std::vector<dmod::array1d> &inputSignals_)
	{
		inputSignals = inputSignals_;
	}

	EKFState GradientTuner::gradDirection(EKFState &step)
	{
		EKFState coef = EKFState();
		for (int i = 0; i < 4; i++)		//state
		{
			if (abs(step.state(i)) > 0.000000001)
			{
				EKFState tmp = EKFState();
				tmp.state(i) += step.state(i);

				float var_plus = FilterTuning::fitness(inputSignals, currentState + tmp);
				float var_minus = FilterTuning::fitness(inputSignals, currentState - tmp);
				coef.state(i) = gradSign(var_plus - var_minus);
			}
		}
		for (int i = 0; i < 4; i++)		//Rw
		{
			for (int j = 0; j < 4; j++)
			{
				if (abs(step.Rw(i, j)) > 0.000000001)
				{
					EKFState tmp = EKFState();
					tmp.Rw(i, j) += step.Rw(i, j);
					float var_plus = FilterTuning::fitness(inputSignals, currentState + tmp);
					float var_minus = FilterTuning::fitness(inputSignals, currentState - tmp);
					coef.Rw(i) = gradSign(var_plus - var_minus);
				}
			}
		}
		for (int i = 0; i < 4; i++)		//R
		{
			for (int j = 0; j < 4; j++)
			{
				if (abs(step.R(i, j)) > 0.000000001)
				{
					EKFState tmp = EKFState();
					tmp.R(i, j) += step.R(i, j);
					float var_plus = FilterTuning::fitness(inputSignals, currentState + tmp);
					float var_minus = FilterTuning::fitness(inputSignals, currentState - tmp);
					coef.R(i) = gradSign(var_plus - var_minus);
				}
			}
		}
		EKFState tmp = EKFState();
		tmp.Rn += step.Rn;
		float var_plus = FilterTuning::fitness(inputSignals, currentState + tmp);
		float var_minus = FilterTuning::fitness(inputSignals, currentState - tmp);
		coef.Rn = gradSign(var_plus - var_minus);
		return coef;
	}


	void GradientTuner::makeStep()
	{
		EKFState coef = gradDirection(step);
		currentState += coef*step;
	}

	EKFState GradientTuner::tune()
	{
		for (int i = 0; i < iterationsCount; i++)
		{
			makeStep();
			std::cout << "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b" << (double)(i + 1) / iterationsCount * 100 << "%%";
			printer::console_print_full_Kalman_state(currentState);
		}
		std::cout << std::endl;
		return currentState;
	}

	int GradientTuner::gradSign(float s, float interval)
	{
		if (s > interval)
			return 1;
		else if (s < -interval)
			return -1;
		else
			return 0;
	}

}
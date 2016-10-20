#include "GradientTuner.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include "SignalAnalysis.h"
#include "StatePrinter.h"
#include "FilterTuning.h"

FilterTuning::GradientTuner::GradientTuner(double **inputSignals_, int signalSize_, int signalsCount_, int iterationsCount_,
	ExtendedKalmanFilterIS1DState currentState_, ExtendedKalmanFilterIS1DState step_)
	: inputSignals(inputSignals_), signalSize(signalSize_), signalsCount(signalsCount_), iterationsCount(iterationsCount_), 
	currentState(currentState_), step(step_) {}


FilterTuning::GradientTuner::~GradientTuner() {}

void FilterTuning::GradientTuner::changeSignals(double **inputSignals_, int signalsCount_)
{
	inputSignals = inputSignals_;
	signalsCount = signalsCount_;
}

void FilterTuning::GradientTuner::makeStep()
{
	//Non adaptive step!
	//Classic variant
	//Step of descent, which is influenced by variations of difference between estimation results and original signals
	ExtendedKalmanFilterIS1DState coef = ExtendedKalmanFilterIS1DState();
	ExtendedKalmanFilterIS1D filter; 
	double *recSignal = new double[signalSize];
	double *difference = new double[signalSize];

	//Estimate current variance
	double current_variance = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState);

	//Estimate varriances with0 descenc
	for (int i = 0; i < 4; i++)		//state
	{
		if (abs(step.state(i)) > 0.000000001)
		{
			ExtendedKalmanFilterIS1DState tmp = ExtendedKalmanFilterIS1DState();
			tmp.state(i) += step.state(i);
			double var_plus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState + tmp);
			double var_minus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState - tmp);
			coef.state(i) = var_plus - var_minus;
		}
	}
	for (int i = 0; i < 4; i++)		//Rw
	{
		for (int j = 0; j < 4; j++)
		{
			if (abs(step.Rw(i, j)) > 0.000000001)
			{
				ExtendedKalmanFilterIS1DState tmp = ExtendedKalmanFilterIS1DState();
				tmp.Rw(i, j) += step.Rw(i, j);
				double var_plus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState + tmp);
				double var_minus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState - tmp);
				coef.Rw(i) = var_plus - var_minus;
			}
		}
	}
	for (int i = 0; i < 4; i++)		//R
	{
		for (int j = 0; j < 4; j++)
		{
			if (abs(step.R(i, j)) > 0.000000001)
			{
				ExtendedKalmanFilterIS1DState tmp = ExtendedKalmanFilterIS1DState();
				tmp.R(i, j) += step.R(i, j);
				double var_plus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState + tmp);
				double var_minus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState - tmp);
				coef.R(i) = var_plus - var_minus;
			}
		}
	}
	ExtendedKalmanFilterIS1DState tmp = ExtendedKalmanFilterIS1DState();
	tmp.Rn += step.Rn;
	double var_plus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState + tmp);
	double var_minus = FilterTuning::fitness(inputSignals, signalsCount, signalSize, currentState - tmp);
	coef.Rn = var_plus - var_minus;

	currentState += coef*step;


	//Debug
	StatePrinter::console_print_full_Kalman_state(coef);
	std::cout << "------------------" << std::endl;
	StatePrinter::console_print_full_Kalman_state(currentState);

	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl ;


	//StatePrinter::console_print_full_Kalman_state(coef);
	//StatePrinter::console_print_full_Kalman_state(currentState);
	//std::cout << "--------------------------------" << std::endl;

	delete[] difference;
	delete[] recSignal;
}

ExtendedKalmanFilterIS1DState FilterTuning::GradientTuner::tune()
{
	for (int i = 0; i < iterationsCount; i++)
		makeStep() ;
	return currentState;
}

int FilterTuning::GradientTuner::gradSign(double s, double interval)
{
	if (s > interval)
		return 1;
	else if (s < -interval)
		return -1;
	else
		return 0;
}
#include "GradientDescentFilterIS1D.h"

GradientDescentFilterIS1D::GradientDescentFilterIS1D(Eigen::Vector4d state_, Eigen::Vector4d step_, int iterNumber_)
	: state(state_), step(step_), iterNumber(iterNumber_) {}

GradientDescentFilterIS1D::GradientDescentFilterIS1D() {}

GradientDescentFilterIS1D::~GradientDescentFilterIS1D() {}

Eigen::Vector4d GradientDescentFilterIS1D::getState()
{
	return state;
}

void GradientDescentFilterIS1D::setStep(Eigen::Vector4d step_)
{
	step = step_;
}

Eigen::Vector4d GradientDescentFilterIS1D::getStep()
{
	return step;
}

void GradientDescentFilterIS1D::setState(Eigen::Vector4d st)
{
	state = st;
}

double GradientDescentFilterIS1D::h(Eigen::Vector4d st)
{
	return st(0) + st(1)*cos(st(3));
}

Eigen::Vector4d GradientDescentFilterIS1D::f(Eigen::Vector4d st)
{
	return st + Eigen::Vector4d(0, 0, 0, 2 * M_PI*st(2));
}

Eigen::Vector4d GradientDescentFilterIS1D::gradient(double obs, Eigen::Vector4d st)
{
	if (obs - h(st) > 0)
		return Eigen::Vector4d(1, cos(st(3)), 0, -st(1)*sin(st(3)));
	else
		return Eigen::Vector4d(1, cos(st(3)), 0, -st(1)*sin(st(3)))*-1;
}

void GradientDescentFilterIS1D::estimate(double obs, StopCriterion criterion, bool doPrediction)
{
	if (doPrediction)
		state = f(state);
	switch (criterion)
	{
	case StopCriterion::FixedIterationsCount:
		for (int i = 0; i < iterNumber; ++i)
		{
			Eigen::Vector4d grad = gradient(obs, state);
			for (int k = 0; k < 4; ++k)
				grad(k) = grad(k)*step(k);
			state += grad;
		}
		break;
	case StopCriterion::AdaptiveStep:
		Eigen::Vector4d diff = step / iterNumber;
		Eigen::Vector4d tmp_step = step;
		for (int i = 0; i < iterNumber; ++i)
		{
			Eigen::Vector4d grad = gradient(obs, state);
			for (int k = 0; k < 4; ++k)
				grad(k) = grad(k)*tmp_step(k);
			state += grad;
			tmp_step -= diff;
		}
		break;
	}
}

double GradientDescentFilterIS1D::evaluateSignalValue()
{
	return h(state);
}

int GradientDescentFilterIS1D::sign(double s)
{
	if (s > 0.00000000001)
		return 1;
	else if (s < -0.00000000001)
		return -1;
	else
		return 0;
}
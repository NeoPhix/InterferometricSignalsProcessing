#include "GradientDescentFilterIS1D.h"

GradientDescentFilterIS1D::GradientDescentFilterIS1D(Eigen::Vector4d state_, Eigen::Vector4d step_, int iterNumber_)
	: state(state_), step(step_), iterNumber(iterNumber_) {}

GradientDescentFilterIS1D::GradientDescentFilterIS1D() {}

GradientDescentFilterIS1D::~GradientDescentFilterIS1D() {}

Eigen::Vector4d GradientDescentFilterIS1D::getState()
{
	return state;
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

Eigen::Vector4d GradientDescentFilterIS1D::gradient(Eigen::Vector4d st)
{
	return Eigen::Vector4d(1, cos(st(3)), 0, -st(1)*sin(st(3)));
}

void GradientDescentFilterIS1D::estimate(double obs, bool doPrediction)
{
	if (doPrediction)
		state = f(state);
	for (int i = 0; i < iterNumber; ++i)
	{
		Eigen::Vector4d grad = gradient(state);
		for (int k = 0; k < 4; ++k)
			grad(k) = sign(grad(k))*step(k);
		state += grad;
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
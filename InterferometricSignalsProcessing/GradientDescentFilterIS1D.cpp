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

//Eigen::RowVector4d GradientDescentFilter1D::Ht(Eigen::Vector4d st)
//{
//	return Eigen::RowVector4d(1, cos(st(3)), 0, -st(1)*sin(st(3)));
//}

void GradientDescentFilterIS1D::estimate(double obs)
{
	//TODO
	//Eigen::Vector4d predict = f(state);
	//Eigen::Matrix4d F = Ft(state);
	//Eigen::Matrix4d Rpr = F*(R*F.transpose()) + Rw*Rw.transpose();
	//Eigen::RowVector4d H = Ht(predict);
	//Eigen::Vector4d P = Rpr*H.transpose() / (H*Rpr*H.transpose() + Rn);
	//state = predict + P*(obs - h(predict));
	//R = (Eigen::Matrix4d::Identity() - P*H)*Rpr;
}

double GradientDescentFilterIS1D::evaluateSignalValue()
{
	return h(state);
}
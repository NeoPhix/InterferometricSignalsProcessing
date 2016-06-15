#include "EKFIneterferometricSignal1D.h"



EKFIneterferometricSignal1D::EKFIneterferometricSignal1D()
{

}


EKFIneterferometricSignal1D::~EKFIneterferometricSignal1D()
{
}

double EKFIneterferometricSignal1D::h(Eigen::Vector4d state)
{
	return state(0) + state(1)*cos(state(3));
}

Eigen::Vector4d EKFIneterferometricSignal1D::f(Eigen::Vector4d state)
{
	return state + Eigen::Vector4d(0, 0, 0, 2 * M_PI*state(2));
}

Eigen::Matrix4d EKFIneterferometricSignal1D::Ft(Eigen::Vector4d state)
{
	double F[] = { 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 2 * M_PI, 1 };
	return Eigen::Matrix4d(F);
}

Eigen::RowVector4d EKFIneterferometricSignal1D::Ht(Eigen::Vector4d state)
{
	return Eigen::RowVector4d(1, cos(state(3)), 0, -state(1)*sin(state(3)));
}
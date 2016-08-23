#include "EKFIneterferometricSignal1D.h"

EKFIneterferometricSignal1D::EKFIneterferometricSignal1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_)
	: state(state_), R(R_), Rw(Rw_), Rn(Rn_) {}


EKFIneterferometricSignal1D::~EKFIneterferometricSignal1D()
{
}

Eigen::Vector4d EKFIneterferometricSignal1D::getState()
{
	return state ;
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

void EKFIneterferometricSignal1D::estimate(double obs)
{
	Eigen::Vector4d predict = f(state);
	Eigen::Matrix4d F = Ft(state);
	Eigen::Matrix4d Rpr = F*R*F.transpose() + Rw*Rw.transpose();
	Eigen::RowVector4d H = Ht(predict);
	Eigen::Vector4d P = Rpr*H.transpose() / (H*Rpr*H.transpose() + Rn);
	state = predict + P*(obs - h(predict));
	R = (Eigen::Matrix4d::Identity()-P*H)*Rpr ;
}
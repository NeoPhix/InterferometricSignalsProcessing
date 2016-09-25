#include "ExtendedKalmanFilterIS1D.h"
#include <iostream>

ExtendedKalmanFilterIS1D::ExtendedKalmanFilterIS1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_)
	: state(state_), R(R_), Rw(Rw_), Rn(Rn_) {}

ExtendedKalmanFilterIS1D::ExtendedKalmanFilterIS1D(ExtendedKalmanFilterIS1DState full_state)
	: state(full_state.state), R(full_state.R), Rw(full_state.Rw), Rn(full_state.Rn) {}

ExtendedKalmanFilterIS1D::ExtendedKalmanFilterIS1D() {}

ExtendedKalmanFilterIS1D::~ExtendedKalmanFilterIS1D()
{
}

Eigen::Vector4d ExtendedKalmanFilterIS1D::getState()
{
	return state ;
}

double ExtendedKalmanFilterIS1D::h(Eigen::Vector4d st)
{
	return st(0) + st(1)*cos(st(3));
}

Eigen::Vector4d ExtendedKalmanFilterIS1D::f(Eigen::Vector4d st)
{
	return st + Eigen::Vector4d(0, 0, 0, 2 * M_PI*st(2));
}

Eigen::Matrix4d ExtendedKalmanFilterIS1D::Ft(Eigen::Vector4d st)
{
	Eigen::Matrix4d F;
	F << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 2 * M_PI, 1;
	return F;
}

Eigen::RowVector4d ExtendedKalmanFilterIS1D::Ht(Eigen::Vector4d st)
{
	return Eigen::RowVector4d(1, cos(st(3)), 0, -st(1)*sin(st(3)));
}

void ExtendedKalmanFilterIS1D::estimate(double obs)
{
	double startPhase = state(3);
	Eigen::Vector4d predict = f(state);
	Eigen::Matrix4d F = Ft(state);
	Eigen::Matrix4d Rpr = F*(R*F.transpose()) + Rw*Rw.transpose();
	Eigen::RowVector4d H = Ht(predict);
	Eigen::Vector4d P =  Rpr*H.transpose() / (H*Rpr*H.transpose() + Rn);
	state = predict + P*(obs - h(predict));
	R = (Eigen::Matrix4d::Identity()-P*H)*Rpr;
	//state(2) = (state(3) - startPhase) / (2 * M_PI );	//New frequency from phase!
}

void ExtendedKalmanFilterIS1D::estimate(double obs, double ph)		//new idea with known phase shift!
{
	double startPhase = state(3);
	Eigen::Vector4d predict = f(state);
	predict(3) = ph;
	Eigen::Matrix4d F = Ft(state);
	Eigen::Matrix4d Rpr = F*(R*F.transpose()) + Rw*Rw.transpose();
	Eigen::RowVector4d H = Ht(predict);
	Eigen::Vector4d P = Rpr*H.transpose() / (H*Rpr*H.transpose() + Rn);
	state = predict + P*(obs - h(predict));
	R = (Eigen::Matrix4d::Identity() - P*H)*Rpr;
	state(2) = (state(3) - startPhase) / (2 * M_PI );	//New frequency from phase!
}

ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1D::getFullState()
{
	ExtendedKalmanFilterIS1DState st = { state, R, Rw, Rn };
	return st ;
}

double ExtendedKalmanFilterIS1D::evaluateSignalValue()
{
	return h(state) ;
}

//State methods
ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1DState::operator+(ExtendedKalmanFilterIS1DState s)
{
	ExtendedKalmanFilterIS1DState res;
	res.state = state + s.state;
	res.Rw = Rw + s.Rw;
	res.R = R + s.R;
	res.Rn = Rn + s.Rn;
	return res;
}

ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1DState::operator-(ExtendedKalmanFilterIS1DState s)
{
	ExtendedKalmanFilterIS1DState res;
	res.state = state - s.state;
	res.Rw = Rw - s.Rw;
	res.R = R - s.R;
	res.Rn = Rn - s.Rn;
	return res;
}

ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1DState::operator*(ExtendedKalmanFilterIS1DState s)
{
	ExtendedKalmanFilterIS1DState res;
	for (int i = 0; i < 4; i++)		//state
	{
		res.state(i) = state(i) * s.state(i) ;
	}
	for (int i = 0; i < 4; i++)		//Rw
	{
		for (int j = 0; j < 4; j++)
		{
			res.Rw(i, j) = Rw(i, j) * s.Rw(i, j);
			res.R(i, j) = R(i, j) * s.R(i, j);
		}
	}
	res.Rn = Rn * s.Rn;	//Rn
	return res;
}

void ExtendedKalmanFilterIS1DState::operator+=(ExtendedKalmanFilterIS1DState s)
{
	state += s.state;
	Rw += s.Rw;
	R += s.R;
	Rn += s.Rn;
}

void ExtendedKalmanFilterIS1DState::operator-=(ExtendedKalmanFilterIS1DState s)
{
	state -= s.state;
	Rw -= s.Rw;
	R -= s.R;
	Rn -= s.Rn;
}

void ExtendedKalmanFilterIS1DState::operator*=(ExtendedKalmanFilterIS1DState s)
{
	for (int i = 0; i < 4; i++)		//state
	{
		state(i) *= s.state(i);
	}
	for (int i = 0; i < 4; i++)		//Rw
	{
		for (int j = 0; j < 4; j++)
		{
			Rw(i, j) *= s.Rw(i, j);
			R(i, j) *= s.R(i, j);
		}
	}
	Rn *= s.Rn;	//Rn
}


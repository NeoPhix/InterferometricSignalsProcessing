#include <iostream>

#include "ExtendedKalmanFilterIS1D.h"

ExtendedKalmanFilterIS1D::ExtendedKalmanFilterIS1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_)
	: state(state_), R(R_), Rw(Rw_), Rn(Rn_) {}

ExtendedKalmanFilterIS1D::ExtendedKalmanFilterIS1D(ExtendedKalmanFilterIS1DState &full_state)
	: state(full_state.state), 
	  R(full_state.R), 
	  Rw(full_state.Rw), 
	  Rn(full_state.Rn) {}

ExtendedKalmanFilterIS1D::ExtendedKalmanFilterIS1D() {}

ExtendedKalmanFilterIS1D::~ExtendedKalmanFilterIS1D() {}

Eigen::Vector4d ExtendedKalmanFilterIS1D::getState()
{
	return state;
}

void ExtendedKalmanFilterIS1D::setState(Eigen::Vector4d st)
{
	state = st;
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
	Eigen::Vector4d predict = f(state);
	Eigen::Matrix4d F = Ft(state);
	Eigen::Matrix4d Rpr = F*(R*F.transpose()) + Rw*Rw.transpose();
	Eigen::RowVector4d H = Ht(predict);
	Eigen::Vector4d P =  Rpr*H.transpose() / (H*Rpr*H.transpose() + Rn);
	state = predict + P*(obs - h(predict));
	R = (Eigen::Matrix4d::Identity()-P*H)*Rpr;
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

double ExtendedKalmanFilterIS1D::evaluateSignalValue(Eigen::Vector4d &st)
{
	return h(st);
}


std::vector<double> ExtendedKalmanFilterIS1D::getRestoredSignal(std::vector<double> &signal)
{
	std::vector<Eigen::Vector4d> states = estimateAll(signal);
	return getRestoredSignal(states);
}

std::vector<double> ExtendedKalmanFilterIS1D::getRestoredSignal(std::vector<Eigen::Vector4d> &states)
{
	size_t N = states.size();
	std::vector<double> restoredSignal(N);
	for (int i = 0; i < N; ++i)
	{
		restoredSignal[i] = evaluateSignalValue(states[i]);
	}
	return restoredSignal;
}

std::vector<Eigen::Vector4d> ExtendedKalmanFilterIS1D::estimateAll(std::vector<double> &signal)
{
	size_t N = signal.size();
	std::vector<Eigen::Vector4d> states(N);
	for (int i = 0; i < N; ++i)
	{
		estimate(signal[i]);
		states[i] = getState();
	}
	return states;
}


//State methods
ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1DState::operator+(ExtendedKalmanFilterIS1DState &s)
{
	ExtendedKalmanFilterIS1DState res;
	res.state = state + s.state;
	res.Rw = Rw + s.Rw;
	res.R = R + s.R;
	res.Rn = Rn + s.Rn;
	return res;
}

ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1DState::operator-(ExtendedKalmanFilterIS1DState &s)
{
	ExtendedKalmanFilterIS1DState res;
	res.state = state - s.state;
	res.Rw = Rw - s.Rw;
	res.R = R - s.R;
	res.Rn = Rn - s.Rn;
	return res;
}

ExtendedKalmanFilterIS1DState ExtendedKalmanFilterIS1DState::operator*(ExtendedKalmanFilterIS1DState &s)
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

ExtendedKalmanFilterIS1DState::ExtendedKalmanFilterIS1DState()
{
	state = Eigen::Vector4d(0,0,0,0);
	Rw <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	R <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	Rn = 0;
}

ExtendedKalmanFilterIS1DState::ExtendedKalmanFilterIS1DState(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_)
	: state(state_), R(R_), Rw(Rw_), Rn(Rn_) {}

ExtendedKalmanFilterIS1DState::~ExtendedKalmanFilterIS1DState()
{
}
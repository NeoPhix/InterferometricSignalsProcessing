#ifndef ExtendedKalmanFilterIS1D_H
#define ExtendedKalmanFilterIS1D_H

#include <Eigen\Dense>

class ExtendedKalmanFilterIS1DState
{
public:
	ExtendedKalmanFilterIS1DState();
	ExtendedKalmanFilterIS1DState(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, float Rn_);
	~ExtendedKalmanFilterIS1DState();

	Eigen::Vector4d state;
	Eigen::Matrix4d R;
	Eigen::Matrix4d Rw;
	float Rn;

	ExtendedKalmanFilterIS1DState operator+(ExtendedKalmanFilterIS1DState s);
	ExtendedKalmanFilterIS1DState operator-(ExtendedKalmanFilterIS1DState s);
	ExtendedKalmanFilterIS1DState operator*(ExtendedKalmanFilterIS1DState s);
	void operator+=(ExtendedKalmanFilterIS1DState s);
	void operator-=(ExtendedKalmanFilterIS1DState s);
	void operator*=(ExtendedKalmanFilterIS1DState s);
};

class ExtendedKalmanFilterIS1D
{
public:
	ExtendedKalmanFilterIS1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, float Rn_);
	ExtendedKalmanFilterIS1D(ExtendedKalmanFilterIS1DState full_state);
	ExtendedKalmanFilterIS1D();
	~ExtendedKalmanFilterIS1D();

	Eigen::Vector4d getState();
	void setState(Eigen::Vector4d st);
	void estimate(float obs);
	ExtendedKalmanFilterIS1DState getFullState() ;
	float evaluateSignalValue() ;
private:
	Eigen::Vector4d state;
	Eigen::Matrix4d R;
	Eigen::Matrix4d Rw;
	float Rn;
	
	float h(Eigen::Vector4d st);
	Eigen::Vector4d f(Eigen::Vector4d st);
	Eigen::Matrix4d Ft(Eigen::Vector4d st);
	Eigen::RowVector4d Ht(Eigen::Vector4d st);
};

#endif


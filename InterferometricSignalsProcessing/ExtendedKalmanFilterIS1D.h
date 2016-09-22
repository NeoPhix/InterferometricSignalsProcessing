#ifndef ExtendedKalmanFilterIS1D_H
#define ExtendedKalmanFilterIS1D_H

#include <Eigen\Dense>

class ExtendedKalmanFilterIS1DState
{
public:
	Eigen::Vector4d state;
	Eigen::Matrix4d R;
	Eigen::Matrix4d Rw;
	double Rn;

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
	ExtendedKalmanFilterIS1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_);
	ExtendedKalmanFilterIS1D(ExtendedKalmanFilterIS1DState full_state);
	ExtendedKalmanFilterIS1D();
	~ExtendedKalmanFilterIS1D();

	Eigen::Vector4d getState();
	void estimate(double obs);
	void estimate(double obs, double ph);		//new idea with known phase shift!
	ExtendedKalmanFilterIS1DState getFullState() ;
	double evaluateSignalValue() ;
private:
	Eigen::Vector4d state;
	Eigen::Matrix4d R;
	Eigen::Matrix4d Rw;
	double Rn;
	
	double h(Eigen::Vector4d st);
	Eigen::Vector4d f(Eigen::Vector4d st);
	Eigen::Matrix4d Ft(Eigen::Vector4d st);
	Eigen::RowVector4d Ht(Eigen::Vector4d st);
};

#endif


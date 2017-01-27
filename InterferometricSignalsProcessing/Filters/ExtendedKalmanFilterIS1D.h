#ifndef ExtendedKalmanFilterIS1D_H
#define ExtendedKalmanFilterIS1D_H

#include <vector>

#include <Eigen\Dense>

class ExtendedKalmanFilterIS1DState
{
public:
	ExtendedKalmanFilterIS1DState();
	ExtendedKalmanFilterIS1DState(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_);
	~ExtendedKalmanFilterIS1DState();

	Eigen::Vector4d state;
	Eigen::Matrix4d R;
	Eigen::Matrix4d Rw;
	double Rn;

	ExtendedKalmanFilterIS1DState operator+(ExtendedKalmanFilterIS1DState &s);
	ExtendedKalmanFilterIS1DState operator-(ExtendedKalmanFilterIS1DState &s);
	ExtendedKalmanFilterIS1DState operator*(ExtendedKalmanFilterIS1DState &s);
	void operator+=(ExtendedKalmanFilterIS1DState s);
	void operator-=(ExtendedKalmanFilterIS1DState s);
	void operator*=(ExtendedKalmanFilterIS1DState s);
};

class ExtendedKalmanFilterIS1D
{
public:
	ExtendedKalmanFilterIS1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_);
	ExtendedKalmanFilterIS1D(ExtendedKalmanFilterIS1DState &full_state);
	ExtendedKalmanFilterIS1D();
	~ExtendedKalmanFilterIS1D();

	Eigen::Vector4d getState();
	void setState(Eigen::Vector4d st);
	void estimate(double obs);
	ExtendedKalmanFilterIS1DState getFullState();
	double evaluateSignalValue();
	double evaluateSignalValue(Eigen::Vector4d &st);

	std::vector<double> getRestoredSignal(std::vector<double> &signal);				
	std::vector<double> getRestoredSignal(std::vector<Eigen::Vector4d> &states);
	std::vector<Eigen::Vector4d> estimateAll(std::vector<double> &signal);	
	//!!!its important to understand the following moment:
	//getRestoredSignal(signal) = getRestoredSignal(estimateAll(signal));
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


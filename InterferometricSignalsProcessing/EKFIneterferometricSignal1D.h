#ifndef EKFIneterferometricSignal1D_H
#define EKFIneterferometricSignal1D_H

#include <Eigen\Dense>

class EKFIneterferometricSignal1D
{
public:
	EKFIneterferometricSignal1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_);
	~EKFIneterferometricSignal1D();

	Eigen::Vector4d getState();
	void estimate(double obs);
private:
	Eigen::Vector4d state;
	Eigen::Matrix4d R;
	Eigen::Matrix4d Rw;
	double Rn;
	
	double h(Eigen::Vector4d state);
	Eigen::Vector4d f(Eigen::Vector4d state);
	Eigen::Matrix4d Ft(Eigen::Vector4d state);
	Eigen::RowVector4d Ht(Eigen::Vector4d state);
};

#endif

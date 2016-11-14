#ifndef GRADIENT_DESCENT_FILTER_1D_H
#define GRADIENT_DESCENT_FILTER_1D_H

#include <Eigen\Dense>

class GradientDescentFilter1D
{
public:
	//GradientDescentFilter1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, double Rn_);
	GradientDescentFilter1D();
	~GradientDescentFilter1D();

	Eigen::Vector4d getState();
	void estimate(double obs);
	double evaluateSignalValue();
private:
	Eigen::Vector4d state;
	Eigen::Vector4d step;
	double Rn;

	double h(Eigen::Vector4d st);
	Eigen::Vector4d f(Eigen::Vector4d st);
	//Eigen::RowVector4d Ht(Eigen::Vector4d st);
};

#endif
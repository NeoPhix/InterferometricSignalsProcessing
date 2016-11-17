#ifndef GradientDescentFilterIS1D_H
#define GradientDescentFilterIS1D_H

#include <Eigen\Dense>

class GradientDescentFilterIS1D
{
public:
	GradientDescentFilterIS1D(Eigen::Vector4d state_, Eigen::Vector4d step_, int iterNumber_);
	GradientDescentFilterIS1D();
	~GradientDescentFilterIS1D();

	Eigen::Vector4d getState();
	void setState(Eigen::Vector4d st);
	void estimate(double obs, bool doPrediction = true);
	double evaluateSignalValue();
private:
	Eigen::Vector4d state;
	Eigen::Vector4d step;
	int iterNumber;

	double h(Eigen::Vector4d st);
	int sign(double s);
	Eigen::Vector4d f(Eigen::Vector4d st);
	Eigen::Vector4d gradient(Eigen::Vector4d st);
};

#endif
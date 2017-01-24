#ifndef GradientDescentFilterIS1D_H
#define GradientDescentFilterIS1D_H

#include <Eigen\Dense>

enum class StopCriterion {FixedIterationsCount, AdaptiveStep};

class GradientDescentFilterIS1D
{
public:
	GradientDescentFilterIS1D(Eigen::Vector4d state_, Eigen::Vector4d step_, int iterNumber_);
	GradientDescentFilterIS1D();
	~GradientDescentFilterIS1D();

	Eigen::Vector4d getState();
	Eigen::Vector4d getStep();
	void setState(Eigen::Vector4d st);
	void setStep(Eigen::Vector4d step_);
	void estimate(double obs, StopCriterion criterion = StopCriterion::AdaptiveStep, bool doPrediction = true);
	double evaluateSignalValue();
private:
	Eigen::Vector4d state;
	Eigen::Vector4d step;
	int iterNumber;

	double h(Eigen::Vector4d st);
	int sign(double s);
	Eigen::Vector4d f(Eigen::Vector4d st);
	Eigen::Vector4d gradient(double obs, Eigen::Vector4d st);
};

#endif
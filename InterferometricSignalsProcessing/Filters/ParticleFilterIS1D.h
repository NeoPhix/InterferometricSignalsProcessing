#ifndef ParticleFilterIS1D_H
#define ParticleFilterIS1D_H

#include <vector>

#include <Eigen/Dense>

class ParticleFilterIS1D
{
public:
	ParticleFilterIS1D(Eigen::Vector4d state_, Eigen::Matrix4d R_, Eigen::Matrix4d Rw_, float Rn_);
	~ParticleFilterIS1D();

	Eigen::Vector4d getState();
	void estimate(float obs);
private:
	std::vector<float> state;
	std::vector<float> means;
	std::vector<float> vars;

	float Rn;

	float h(Eigen::Vector4d state);
	Eigen::Vector4d f(Eigen::Vector4d state);
	Eigen::Matrix4d Ft(Eigen::Vector4d state);
	Eigen::RowVector4d Ht(Eigen::Vector4d state);
};

#endif


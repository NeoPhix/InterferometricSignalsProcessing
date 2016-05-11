#ifndef INTERFEROMETRIC_SYSTEM_1D_H
#define INTERFEROMETRIC_SYSTEM_1D_H

#include <Eigen/Dense>

#include "IDynamicSystem.h"

//State contains Background, Amplitude, frequency, and Phase components
class InterferometricSystem1D :	public IDynamicSystem<Eigen::Vector4d, double, Eigen::Matrix4d, Eigen::RowVector4d>
{
public:
	InterferometricSystem1D();
	~InterferometricSystem1D();
private:

};

#endif


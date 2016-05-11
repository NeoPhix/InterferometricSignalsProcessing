#ifndef EXTNDED_KALMAN_FILTER_H
#define EXTNDED_KALMAN_FILTER_H

#include <Eigen/Dense>

template <class System>
class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter(System *SYSTEM);
	~ExtendedKalmanFilter();

private:
	System system ;
};

#endif
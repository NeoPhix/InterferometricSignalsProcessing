#ifndef EXTNDED_KALMAN_FILTER_H
#define EXTNDED_KALMAN_FILTER_H

#include <opencv\cv.h>
#include "IFilter.h"
#include "IDynamicSystem.h"

class ExtendedKalmanFilter : public IFilter
{
public:
	ExtendedKalmanFilter();
	~ExtendedKalmanFilter();

private:

};

#endif
#include <iostream>
#include <ctime>
#include <functional>
#include <cmath>

#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "ExtendedKalmanFilter.h"




//template <class State, class Obs, class StateDer, class ObsDer>
//class ExtendedKalmanFilter
//{
//public:
//	ExtendedKalmanFilter(std::function<State(State)> trans,
//		std::function<StateDer(State)> transDer,
//		std::function<Obs(State)> obs,
//		std::function<ObsDer(State)> obsDer,
//		State st) ;
//	//~ExtendedKalmanFilter();
//
//	//State operator()(Observation obs);
//private:
//	std::function<State(State)> translate;
//	std::function<StateDer(State)> getTranslationDerivative;
//	std::function<Obs(State)> observe;
//	std::function<ObsDer(State)> getObservationDerivative;
//	State state;
//	Obs obs;
//};


double h(Eigen::Vector4d state) ;
Eigen::Vector4d f(Eigen::Vector4d state);
Eigen::Matrix4d Ft(Eigen::Vector4d state);
Eigen::RowVector4d Ht(Eigen::Vector4d state);

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;
	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl ;

	Eigen::Vector4d beginState(127, 50, 3, 0) ;
	ExtendedKalmanFilter<Eigen::Vector4d, double, Eigen::Matrix4d, Eigen::RowVector4d> EKF(f, Ft, h, Ht, beginState);
	
	std::cin >> argc ;
	return 0 ;
}



double h(Eigen::Vector4d state)
{
	return state(0) + state(1)*cos(state(3));
}

Eigen::Vector4d f(Eigen::Vector4d state)
{
	return state + Eigen::Vector4d(0, 0, 0, 2 * M_PI*state(2));
}

Eigen::Matrix4d Ft(Eigen::Vector4d state)
{
	double F[] = { 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 2 * M_PI, 1 };
	return Eigen::Matrix4d(F);
}

Eigen::RowVector4d Ht(Eigen::Vector4d state)
{
	return Eigen::RowVector4d(1, cos(state(3)), 0, -state(1)*sin(state(3)));
}

//Eigen::Matrix2d *m2 = new Eigen::Matrix2d;
//Eigen::Matrix2d &m = *m2;
//
//time_t time = clock();
//
//for (long long j = 0; j < 10000; j++)
//{
//	for (long long i = 0; i < 10000; i++)
//	{
//		if (i % 2 == 1)
//		{
//			m(0, 0) += i*j;
//			m(0, 1) += i;
//			m(1, 0) += i;
//			m(1, 1) += i;
//		}
//		else
//		{
//			m(0, 0) += j;
//			m(0, 1) += j;
//			m(1, 0) += j;
//			m(1, 1) += j;
//		}
//	}
//}
//
//std::cout << "Forward = " << clock() - time << std::endl;
//
//time = clock();
//
//for (long long j = 0; j < 10000; j++)
//{
//	for (long long i = 0; i < 10000; i++)
//	{
//		if (i % 2 == 1)
//		{
//			(*m2)(0, 0) += i*j;
//			(*m2)(0, 1) += i;
//			(*m2)(1, 0) += i;
//			(*m2)(1, 1) += i;
//		}
//		else
//		{
//			(*m2)(0, 0) += j;
//			(*m2)(0, 1) += j;
//			(*m2)(1, 0) += j;
//			(*m2)(1, 1) += j;
//		}
//	}
//}
//
//std::cout << "Pointers = " << clock() - time << std::endl;
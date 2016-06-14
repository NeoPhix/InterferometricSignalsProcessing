#include <iostream>
#include <ctime>
#include <functional>
#include <cmath>

#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "EKFIneterferometricSignal1D.h"


int main(int argc, char **argv)
{
	std::cout << argc << std::endl;
	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl ;

	Eigen::Vector4d beginState(127, 50, 3, 0) ;
	
	std::cin >> argc ;
	return 0 ;
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
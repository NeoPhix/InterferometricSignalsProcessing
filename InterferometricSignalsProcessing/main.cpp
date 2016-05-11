#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Dense>
#include <time.h>
#include <functional>

#include "InterferometricSystem1D.h"

//Test functions

//template <class State, class Observation>
//class MyFilter
//{
//public:
//	std::function Translate ;
//	std::function Observe ;
//private:
//	State state ;
//	Observation observation ;
//};

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;

	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl ;

	Eigen::Matrix2d m ;
	Eigen::Matrix2d *m2 = &m ;
	
	time_t time = clock() ;

	for (long long j = 0; j < 10000000000000000000; j++)
	{
		for (long long i = 0; i < 10000000000000000000; i++)
		{
			m(0, 0) = i*j;
			m(0, 1) = i;
			m(1, 0) = i;
			m(1, 1) = i;
		}
	}
	
	std::cout << "Forward = " << clock() - time << std::endl;

	time = clock();

	for (long long j = 0; j < 10000000000000000000; j++)
	{
		for (long long i = 0; i < 10000000000000000000; i++)
		{
			(*m2)(0, 0) = i*j;
			(*m2)(0, 1) = i;
			(*m2)(1, 0) = i;
			(*m2)(1, 1) = i;
		}
	}

	std::cout << "Pointers = " << clock() - time << std::endl;

	std::cin >> argc ;
	return 0 ;
}

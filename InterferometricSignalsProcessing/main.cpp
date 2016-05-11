#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Dense>
#include <time.h>
#include <functional>

#include "InterferometricSystem1D.h"

//Test functions

template <class State, class Observation>
class MyFilter
{
public:
	std::function Translate ;
	std::function Observe ;
private:
	State state ;
	Observation observation ;
};

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;

	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl ;

	

	std::cin >> argc ;
	return 0 ;
}

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <functional>
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
	std::cout << argc << std::endl;

	for (int i = 0; i < argc; i++)
		std::cout << argv[i] << std::endl ;

	std::vector<double> myVector = {1, 1, 1, 1} ;
	for (int i = 0; i < myVector.size(); i++)
		std::cout << myVector[i] << " ";

	cv::Mat myMat(4, 4, CV_32FC1) ; 

	std::cin >> argc ;
	return 0 ;
}
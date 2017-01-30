#include <sstream>
#include <iostream>
#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> //to think why this headers

#include "Tomogram.h"

namespace dmod
{

	Tomogram::Tomogram()
		: d(0), h(0), w(0)
	{
		data = createArray3d(d, h, w);
	}

	Tomogram::Tomogram(size_t d_, size_t h_, size_t w_)
		: d(d_), h(h_), w(w_)
	{
		data = createArray3d(d, h, w);
	}

	Tomogram::Tomogram(const char *path, const char *type, size_t number)
		: d(number), h(0), w(0)
	{

		//for (d = 1; ; ++d)
		//{
		//	std::stringstream str;
		//	str << d << type;
		//	if (INVALID_FILE_ATTRIBUTES == GetFileAttributesA(str.str().c_str()))
		//		break;
		//}

		data = createArray3d(d, h, w);
	}

	Tomogram::~Tomogram()
	{
	}

	void Tomogram::loadImageSequence( const char *path,
									  const char *type,
									  size_t begin_number,
									  size_t end_number,
									  Plane plane )
	{
		for (size_t n = begin_number; n <= end_number; ++n)
		{
			std::stringstream str;
			str << path << n << type;

			if (INVALID_FILE_ATTRIBUTES == GetFileAttributesA(str.str().c_str()))
			{
				std::cout << "There is no image in the directory" << std::endl;
				return;
			}
			cv::Mat mat = cv::imread(str.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			std::cout << n << std::endl;
			array2d arr = array2dFromMat(mat);
			setSignal2D(n - begin_number, arr, plane);
		}
	}

	void Tomogram::initSizeFromImageSequence( const char *path,
											  const char *type,
											  size_t begin_number,
											  size_t end_number,
											  Plane plane )
	{
		std::stringstream str;
		str << path << begin_number << type;
		if (INVALID_FILE_ATTRIBUTES == GetFileAttributesA(str.str().c_str()))
		{
			std::cout << "There is no image in the directory" << std::endl;
			return;
		}
		cv::Mat mat = cv::imread(str.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);

		switch (plane)
		{
		case Plane::XZ:
			w = mat.cols;
			h = end_number - begin_number + 1;
			d = mat.rows;
			break;
		case Plane::XY:
			w = mat.cols;
			h = mat.rows;
			d = end_number - begin_number + 1;
			break;
		case Plane::YZ:
			w = end_number - begin_number + 1;
			h = mat.cols;
			d = mat.rows;
			break;
		}
		data = createArray3d(d, h, w);
	}

	void Tomogram::setSignal1D( size_t x, 
								size_t y, 
								size_t z, 
								array1d &s, 
								Axis axis)
	{
		if (x >= w || x < 0 || y >= h || y < 0 || z >= d || z < 0)
		{
			std::cout << "Error! Array sizes are mismatched!" << std::endl;
			return;
		}
		switch (axis)
		{
		case Axis::X:
			if (s.size() != w)
			{
				std::cout << "Error! Array sizes are mismatched!" << std::endl;
				break;
			}
			data[z][y] = s;
			break;
		case Axis::Y:
			if (s.size() != h)
			{
				std::cout << "Error! Array sizes are mismatched!" << std::endl;
				break;
			}
			for (y = 0; y < h; ++y)
			{
				data[z][y][x] = s[y];
			}
			break;
		case Axis::Z:
			if (s.size() != d)
			{
				std::cout << "Error! Array sizes are mismatched!" << std::endl;
				break;
			}
			for (z = 0; z < d; ++z)
			{
				data[z][y][x] = s[z];
			}
			break;
		}
	}

	void Tomogram::setSignal2D(size_t n, array2d &s, Plane plane)		//XZ = y number,  
	{
		if (isEmpty(s))
		{
			std::cout << "Array is empty!" << std::endl;
			return;
		}
		switch (plane)
		{
		case Plane::XZ:
			if (s.size() != d || s[0].size() != w)
			{
				std::cout << "Error! Array sizes are mismatched!" << std::endl;
				break;
			}
			for (int z = 0; z < d; ++z)
			{
				data[z][n] = s[z];
			}
			break;
		case Plane::XY:
			if (s.size() != h || s[0].size() != w)
			{
				std::cout << "Error! Array sizes are mismatched!" << std::endl;
				break;
			}
			data[n] = s;
			break;
		case Plane::YZ:
			if (s.size() != d || s[0].size() != h)
			{
				std::cout << "Error! Array sizes are mismatched!" << std::endl;
				break;
			}
			for (int z = 0; z < d; ++z)
			{
				for (int y = 0; y < h; ++y)
				{
					data[z][y][n] = s[z][y];
				}
			}
			break;
		}
	}

	void Tomogram::setSignal3D(array3d data_)
	{
		data = data_;
	}

	array1d Tomogram::getSignal1D(size_t x, size_t y, size_t z, Axis axis)		//X, Y, Z
	{
		array1d res;
		switch (axis)
		{
		case Axis::X:
			return data[z][y];
			break;
		case Axis::Y:
			res.resize(h);
			for (y = 0; y < h; ++y)
			{
				res[y] = data[z][y][x];
			}
			break;
		case Axis::Z:
			res.resize(d);
			for (z = 0; z < d; ++z)
			{
				res[z] = data[z][y][x];
			}
			break;
		}
		return std::move(res);
	}

	array2d Tomogram::getSignal2D(size_t n, Plane plane)		//XZ = y number,  
	{
		array2d res;
		switch (plane)
		{
		case Plane::XZ:
			res = createArray2d(d, w);
			for (int z = 0; z < d; ++z)
			{
				res[z] = data[z][n];
			}
			break;
		case Plane::XY:
			return data[n];
			break;
		case Plane::YZ:
			res = createArray2d(d, h);
			for (int z = 0; z < d; ++z)
			{
				for (int y = 0; y < h; ++y)
				{
					res[z][y] = data[z][y][n];
				}
			}
			break;
		}
		return std::move(res);
	}

	array3d Tomogram::getSignal3D()
	{
		return data;
	}

	size_t Tomogram::getDepth()
	{
		return d;
	}

	size_t Tomogram::getHeight()
	{
		return h;
	}

	size_t Tomogram::getWidth()
	{
		return w;
	}
}

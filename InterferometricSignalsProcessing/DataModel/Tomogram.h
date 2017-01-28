#ifndef TOMOGRAM_H
#define TOMOGRAM_H

#include <opencv/cv.h>

#include "SignalAnalysis.h"

namespace dmod
{
	enum Axis { X, Y, Z };		//For 1D signals getting
	enum Plane { XZ, XY, YZ };	//For 2D signals getting

	class Tomogram
	{
	public:
		Tomogram();
		Tomogram(size_t d_, size_t h_, size_t w_);
		Tomogram(const char *path, const char *type, size_t number);
		~Tomogram();

		void loadImageSequence(const char *path, const char *type, size_t number, Plane plane = Plane::XY);

		void setSignal1D(size_t x, size_t y, size_t z, array1d &s, Axis axis = Axis::Z);		//X, Y, Z
		void setSignal2D(size_t n, array2d &s, Plane plane = Plane::XZ);			//XZ = y number,  
		void setSignal3D(array3d data_);

		array1d getSignal1D(size_t x, size_t y, size_t z, Axis axis = Axis::Z);		//X, Y, Z
		array2d getSignal2D(size_t n, Plane plane = Plane::XZ);		//XZ = y number,  
		array3d getSignal3D();
	private:
		size_t d;
		size_t h;
		size_t w;

		array3d data;
	};
}
#endif
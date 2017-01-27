#ifndef TOMOGRAM_H
#define TOMOGRAM_H

#include "SignalAnalysis.h"

namespace dmod
{
	enum Axis { X, Y, Z };		//For 1D signals getting
	enum Plane { XZ, XY, YZ };	//For 2D signals getting

	class Tomogram
	{
	public:
		Tomogram();
		Tomogram(int w_, int h_, int d_);
		Tomogram(const char *path, const char *type, int count);

		~Tomogram();

		void setSignal1D(int n, int m, Axis asix = Axis::Z);		//X, Y, Z
		void setSignal2D(int n, Plane plane = Plane::XZ);			//XZ = y number,  
		void setSignal3D(array3d data_);

		array1d getSignal1D(int n, int m, Axis asix = Axis::Z);	//X, Y, Z
		array2d getSignal2D(int n, Plane plane = Plane::XZ);		//XZ = y number,  
		array3d getSignal3D();

	private:
		int w;
		int h;
		int d;

		array3d data;
	};
}
#endif
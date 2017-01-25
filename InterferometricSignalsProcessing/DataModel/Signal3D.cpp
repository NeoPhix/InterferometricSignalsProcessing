#include <sstream>
#include <iostream>
#include <windows.h>
#include <opencv\cv.h>

#include "Signal3D.h"

Signal3D::Signal3D()
	: w(0), h(0), d(0)
{}

Signal3D::Signal3D(const char *path, const char *type)
	: w(0), h(0), d(0)
{

	//for (d = 1; ; ++d)
	//{
	//	std::stringstream str;
	//	str << d << type;
	//	if (INVALID_FILE_ATTRIBUTES == GetFileAttributesA(str.str().c_str()))
	//		break;
	//}

	data = vector3d(w, vector2d(h, vector1d(d)));
}


Signal3D::~Signal3D()
{
}

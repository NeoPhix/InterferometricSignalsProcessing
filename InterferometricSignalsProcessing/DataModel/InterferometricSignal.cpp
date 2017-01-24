#include <sstream>
#include <iostream>
#include <windows.h>
#include <opencv\cv.h>

#include "InterferometricSignal.h"

InterferometricSignal::InterferometricSignal()
	: w(0), h(0), d(0)
{}

InterferometricSignal::InterferometricSignal(const char *path, const char *type)
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

InterferometricSignal::InterferometricSignal(const char *path, const char *type)
{
}

InterferometricSignal::~InterferometricSignal()
{
}

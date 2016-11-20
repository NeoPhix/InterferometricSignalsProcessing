#include <sstream>
#include <iostream>
#include <windows.h>
#include <opencv\cv.h>

#include "InterferometricSignal.h"

InterferometricSignal::InterferometricSignal(const char *path, const char *type)
{
	int w = 0;
	int h = 0;
	int d = 0;

	for  (d = 1; ; ++d)
	{
		std::stringstream str;
		str << d << type;
		if (INVALID_FILE_ATTRIBUTES == GetFileAttributesA(str.str().c_str()))
			break;
	}


	data = vector3d(w, vector2d(h, vector1d(d)));
}


InterferometricSignal::~InterferometricSignal()
{
}

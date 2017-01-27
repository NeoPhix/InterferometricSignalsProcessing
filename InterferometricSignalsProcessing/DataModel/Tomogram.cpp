#include <sstream>
#include <iostream>
#include <windows.h>
#include <opencv\cv.h>

#include "Tomogram.h"

using namespace dmod;

Tomogram::Tomogram()
	: w(0), h(0), d(0)
{}

Tomogram::Tomogram(const char *path, const char *type, int count)
	: w(0), h(0), d(count)
{

	//for (d = 1; ; ++d)
	//{
	//	std::stringstream str;
	//	str << d << type;
	//	if (INVALID_FILE_ATTRIBUTES == GetFileAttributesA(str.str().c_str()))
	//		break;
	//}

	data = array3d(w, array2d(h, array1d(d)));
}


Tomogram::~Tomogram()
{
}

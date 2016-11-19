#ifndef INTERFEROMETRIC_SIGNAL_H
#define INTERFEROMETRIC_SIGNAL_H

#include <vector>

typedef std::vector<std::vector<std::vector<double>>> vector3d;
typedef std::vector<std::vector<double>> vector2d;
typedef std::vector<double> vector1d;

class InterferometricSignal
{
public:
	InterferometricSignal(const char *path, const char *type);
	~InterferometricSignal();

	vector1d getSignal1D();
	//vector2d get

private:
	vector3d data;
};

#endif
#ifndef INTERFEROMETRIC_SIGNAL_H
#define INTERFEROMETRIC_SIGNAL_H

#include <vector>

typedef std::vector<std::vector<std::vector<float>>> vector3d;
typedef std::vector<std::vector<float>> vector2d;
typedef std::vector<float> vector1d;

class InterferometricSignal
{
public:
	InterferometricSignal();
	InterferometricSignal(const char *path, const char *type);
	~InterferometricSignal();

	void readSignal1D();
	void readSignal2D();
	void 

	vector1d getSignal1D();
	vector2d getSignal2D();


private:
	int w;
	int h;
	int d;

	vector3d data;


};

#endif
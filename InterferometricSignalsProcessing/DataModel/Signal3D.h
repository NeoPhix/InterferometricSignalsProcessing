#ifndef SIGNAL_3D_H
#define SIGNAL_3D_H

#include <vector>

typedef std::vector<std::vector<std::vector<float>>> vector3d;
typedef std::vector<std::vector<float>> vector2d;
typedef std::vector<float> vector1d;

enum Axis {X, Y, Z};	//For 1D signals getting
enum Plane {XY, XZ};	//For 2D signals getting

class Signal3D
{
public:
	Signal3D();
	Signal3D(const char *path, const char *type);

	~Signal3D();

	void readSignal1D();
	void readSignal2D();
	void readSignal3D();

	vector1d getSignal1D();
	vector2d getSignal2D();
	vector3d getSignal3D();

private:
	int w;
	int h;
	int d;

	vector3d data;
};

#endif
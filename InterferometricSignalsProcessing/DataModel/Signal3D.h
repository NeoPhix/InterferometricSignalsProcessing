#ifndef SIGNAL_3D_H
#define SIGNAL_3D_H

#include <vector>

typedef std::vector<std::vector<std::vector<float>>> vector3d;
typedef std::vector<std::vector<float>> vector2d;
typedef std::vector<float> vector1d;

enum Axis {X, Y, Z};		//For 1D signals getting
enum Plane {XZ, XY, YZ};	//For 2D signals getting

class Signal3D
{
public:
	Signal3D();
	Signal3D(int w_, int h_, int d_);
	Signal3D(const char *path, const char *type, int count);

	~Signal3D();

	void setSignal1D(int n, int m, Axis asix = Axis::Z);		//X, Y, Z
	void setSignal2D(int n, Plane plane = Plane::XZ);			//XZ = y number,  
	void setSignal3D(vector2d data_);	

	vector1d getSignal1D(int n, int m, Axis asix = Axis::Z);	//X, Y, Z
	vector2d getSignal2D(int n, Plane plane = Plane::XZ);		//XZ = y number,  
	vector3d getSignal3D();

private:
	int w;
	int h;
	int d;

	vector3d data;
};

#endif
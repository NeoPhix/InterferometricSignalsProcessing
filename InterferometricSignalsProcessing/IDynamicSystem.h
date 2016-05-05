#ifndef IDYNAMIC_SYSTEM_H
#define IDYNAMIC_SYSTEM_H

#include <vector>

class IDynamicSystem
{
public:
	IDynamicSystem();
	~IDynamicSystem();
	virtual void Translate() = 0 ;
	virtual std::vector<double> Observe() = 0 ;
private:
	std::vector<double> State;
};

#endif


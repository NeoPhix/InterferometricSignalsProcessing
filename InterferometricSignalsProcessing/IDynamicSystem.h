#ifndef IDYNAMIC_SYSTEM_H
#define IDYNAMIC_SYSTEM_H

#include <vector>

class IDynamicSystem
{
public:
	virtual void Translate() = 0 ;									//Translate to the next state
	virtual std::vector<double>& Observe() const = 0 ;				//Get observation from current state of the system
	//virtual void 
private:
	std::vector<double> State;
};

#endif


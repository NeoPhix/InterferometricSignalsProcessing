#ifndef DYNAMIC_SYSTEM
#define DYNAMIC_SYSTEM

#include <functional>

template <class STATE, class OBSERVATION, >
class DynamicSystem
{
public:
	DynamicSystem(std::function &translate, std::function &observe);
	~DynamicSystem();
	std::function Translate ;
	std::function Observe ;
};

#endif


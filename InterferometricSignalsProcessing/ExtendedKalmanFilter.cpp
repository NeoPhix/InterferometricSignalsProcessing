#include "ExtendedKalmanFilter.h"

template <class State, class Obs, class StateDer, class ObsDer>
ExtendedKalmanFilter<State, Obs, StateDer, ObsDer>::ExtendedKalmanFilter(std::function<State(State)> trans,
	std::function<StateDer(State)> transDer,
	std::function<Obs(State)> obs,
	std::function<ObsDer(State)> obsDer,
	State st)
{
}
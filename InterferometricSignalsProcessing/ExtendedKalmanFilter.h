#ifndef EXTNDED_KALMAN_FILTER_H
#define EXTNDED_KALMAN_FILTER_H

#include <Eigen/Dense>

template <class State, class Obs, class StateDer, class ObsDer>
class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter(std::function<State(State)> trans,
		std::function<Obs(State)> obs,
		std::function<StateDer(State)> transDer,
		std::function<ObsDer(State)> obsDer,
		State st)
		: translate(trans), observe(obs), getTranslationDerivative(transDer), getObservationDerivative(obsDer), state(st) {}
	//~ExtendedKalmanFilter();

	//State operator()(Observation obs);
private:
	std::function<State(State)> translate;
	std::function<StateDer(State)> getTranslationDerivative;
	std::function<Obs(State)> observe;
	std::function<ObsDer(State)> getObservationDerivative;
	State state;
	Obs obs;
};

#endif
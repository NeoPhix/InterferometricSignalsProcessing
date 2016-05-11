#ifndef IDYNAMIC_SYSTEM_H
#define IDYNAMIC_SYSTEM_H

template <class State, class Observation, class TranslateDerivative, class ObserveDerivative>
class IDynamicSystem
{
public:
	virtual void Translate() = 0 ;										//Translate to the next state
	virtual Observation GetObservation() const = 0 ;					//Get observation from current state of the system
	virtual TranslateDerivative GetTranslateDerivative() const = 0 ;	//Derivatives fo Extended Kalman filter and its modifications
	virtual ObserveDerivative GetObservationDerivative() const = 0 ;
private:
	State state ;
};

#endif


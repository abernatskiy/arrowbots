#ifndef ARROWBOT_SIMULATOR_H
#define ARROWBOT_SIMULATOR_H

#include <vector>
#include <tuple>

#include <boost/numeric/ublas/matrix.hpp>

#include "evclib/ann/direct.h"

/* Class for evaluating Arrowbot controllers. Submit the controller with wire(),
   then run the simulation and evaluation computation with evaluateController().
   The simulation is implemented using GSL's ODE capabilities.
 */
                                                                                

using namespace boost::numeric::ublas;

typedef struct ArrowbotSimulatorParameters
{
	int segments;
	std::vector<std::vector<double>> targetOrientations;
	matrix<double> sensorAttachment;
} ArrowbotSimulatorParameters;

class ArrowbotSimulator
{
	private:

	const ArrowbotSimulatorParameters* const parameters;
	ANNDirect* currentController;
	matrix<double> phiCoefficient, psiCoefficient;

	void validateController();
	void parseController(matrix<double>& W, matrix<double>& Y);

	double evaluateControllerForOrientations(int orientationsIdx);

	public:

	ArrowbotSimulator(const ArrowbotSimulatorParameters* p) : parameters(p), currentController(nullptr){};
	void wire(ANNDirect* newController){currentController = newController;};
	void evaluateController();
	int segments(){return parameters.segments;} const;
};

#endif // ARROWBOT_SIMULATOR_H

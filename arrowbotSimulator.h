#ifndef ARROWBOT_SIMULATOR_H
#define ARROWBOT_SIMULATOR_H

#include <boost/numeric/ublas/matrix.hpp>

#include "evclib/ann/direct.h"

/* Class for evaluating Arrowbot controllers. Submit the controller with wire(),
   then run the simulation and evaluation computation with evaluateController().
   The simulation is implemented using GSL's ODE capabilities.
 */

using namespace boost::numeric::ublas;

typedef struct ArrowbotParameters
{
	int segments;
	matrix<double> sensorAttachment;
} ArrowbotParameters;

typedef struct ArrowbotSimulationParameters
{
	double totalTime;
	double timeStep;
	vector<vector<double>> targetOrientations;
	vector<vector<double>> initialConditions;
} ArrowbotSimulationParameters;

class ArrowbotSimulator
{
	private:

	const ArrowbotParameters* const botParameters;
	const ArrowbotSimulationParameters* const simulationParameters;
	ANNDirect* currentController;
	matrix<double> phiCoefficient, psiCoefficient;

	void validateArrowbotParameters();
	void validateArrowbotSimulationParameters();
	void validateController();
	void parseController(matrix<double>& W, matrix<double>& Y);
	double evaluateControllerForOrientations(int orientationsIdx);

	public:

	ArrowbotSimulator(const ArrowbotParameters* p, const ArrowbotSimulationParameters* sp) :
		parameters(p),
		simulationParameters(sp),
		currentController(nullptr)
	{
		validateArrowbotParameters();
		validateArrowbotSimulationParameters();
	};
	void wire(ANNDirect* newController){currentController = newController;};
	void evaluateController();
	inline int segments(){return botParameters.segments;} const;
};

#endif // ARROWBOT_SIMULATOR_H

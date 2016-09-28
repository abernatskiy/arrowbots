#ifndef ARROWBOT_SIMULATOR_H
#define ARROWBOT_SIMULATOR_H

#include <iostream>
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

std::ostream& operator<<(std::ostream&, const ArrowbotParameters&);

typedef struct ArrowbotSimulationParameters
{
	double totalTime;
	double timeStep;
	vector<vector<double>> targetOrientations;
	vector<vector<double>> initialConditions;
} ArrowbotSimulationParameters;

std::ostream& operator<<(std::ostream&, const ArrowbotSimulationParameters&);

class ArrowbotSimulator
{
	private:

	const ArrowbotParameters& botParameters;
	const ArrowbotSimulationParameters& simParameters;
	ANNDirect* currentController;
	matrix<double> phiCoefficient, psiCoefficient;

	void validateArrowbotParameters();
	void validateArrowbotSimulationParameters();
	void validateController();
	void parseController(matrix<double>& W, matrix<double>& Y);
	double evaluateControllerForOrientations(int orientationsIdx);

	public:

	ArrowbotSimulator(const ArrowbotParameters& p, const ArrowbotSimulationParameters& sp) :
		botParameters(p),
		simParameters(sp),
		currentController(nullptr)
	{
		validateArrowbotParameters();
		validateArrowbotSimulationParameters();
	};
	void wire(ANNDirect* newController);
	void evaluateController();
	inline unsigned segments(){return botParameters.segments;};
};

#endif // ARROWBOT_SIMULATOR_H

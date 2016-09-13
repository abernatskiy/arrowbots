#ifndef ARROWBOT_SIMULATOR_H
#define ARROWBOT_SIMULATOR_H

#include <vector>
#include "evclib/ann/direct.h"

/* Class for evaluating Arrowbot controllers. Submit the controller with wire(),
   then run the simulation and evaluation computation with evaluateController().
   The simulation is implemented using GSL's ODE capabilities.
 */
                                                                                

typedef struct ArrowbotSimulatorParameters
{
	int segments;
	std::vector<std::vector<double>> targetOrientations;
} ArrowbotSimulatorParameters;

class ArrowbotSimulator
{
	private:

	const ArrowbotSimulatorParameters* const parameters;
	double evaluateControllerForOrientations(int orientationsIdx);

	public:

	ArrowbotSimulator(const ArrowbotSimulatorParameters* p);
	void wire(ANNDirect* newController){currentController = newController;};
	void evaluateController();

	int segments(){return parameters.segments;} const;
};

#endif // ARROWBOT_SIMULATOR_H

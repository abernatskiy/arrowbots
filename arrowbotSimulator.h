#ifndef ARROWBOT_SIMULATOR_H
#define ARROWBOT_SIMULATOR_H

#include <string>
#include "evclib/ann/direct.h"

/* Class which handles the simulation of arrowbots via solving ODEs governing them */

class ArrowbotSimulator
{
	private:

	ANNDirect* currentController;

	public:

	ArrowbotSimulator(); // specify environmental params around here
	void wire(ANNDirect* newController){currentController = newController;};
	void evaluate(){currentController->eval = 0.0;};
};

#endif // ARROWBOT_SIMULATOR_H

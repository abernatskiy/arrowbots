#include <iostream>
#include <cstdlib>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "arrowbotSimulator.h"

/* Auxiliary functions */

gsl_matrix? lowerTriangularOnes(int size)
{
	
}

/* ArrowbotSimulator class definitions */

double ArrowbotSimulator::evaluateControllerForOrientations(int orientationsIdx)
{
	auto itOrientation = parameters.begin() + orientationIdx;

	/* ... */
	return 0.;
}

void ArrowbotSimulator::wire(ANNDirect* newController)
{
	currentController = newController;
	
}

void ArrowbotSimulator::evaluateController()
{
	if(!currentController)
	{
		std::cerr << "Evaluation of an unwired Arrowbot attempted, exiting\n";
		exit(EXIT_FAILURE);
	}

	int numEnv = parameters.targetOrientations.size();
	double evalSum = 0.0;
	for(int i=0; i<numEnv; i++)
		evalSum += evaluateControllerForOrientations(i);

	return evalSim/((double) evalSim);
}

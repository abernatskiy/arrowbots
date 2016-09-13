#include <iostream>
#include <cstdlib>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "arrowbotSimulator.h"

typedef struct ArrowbotODEParameters
{
	
} ArrowbotODEParameters;

/* Auxiliary functions */

gsl_matrix? lowerTriangularOnes(int size)
{
	
}

/* Functions needed for the GSL integrator */
int ArrowbotODE(double t, const double y[], double f[], void *params)
{
	ArrowbotSimulator* as = (ArrowbotSimulator*) params;
	return GSL_SUCCESS;
}

int ArrowbotODEJacobian(double t, const double y[], double *dfdy, double dfdt[], void *params)
{
	ArrowbotSimulator* as = (ArrowbotSimulator*) params;
	gsl_matrix_view dfdy_mat = gsl_matrix_view_array(dfdy, 2, 2);
	return GSL_SUCCESS;
}

/* ArrowbotSimulator class definitions */

ArrowbotsSimulator::ArrowbotSimulator(const ArrowbotSimulatorParameters* p) :
	parameters(p),
	currentController(nullptr)
{
	
}

double ArrowbotSimulator::evaluateControllerForOrientations(int orientationsIdx)
{
	
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

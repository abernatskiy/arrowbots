#include <iostream>
#include <cstdlib>

#include <boost/numeric/ublas/io.hpp>

#include "arrowbotSimulator.h"

/* Auxiliary functions */

void lowerTriangularOnes(matrix<double>& K, int size)
{
	K.resize(size, size);
	for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
			K(i,j) = i>=j ? 1. : 0.;
}

/* ArrowbotSimulator class definitions */

// Private

double ArrowbotSimulator::evaluateControllerForOrientations(int orientationsIdx)
{
	auto itOrientation = parameters.begin() + orientationIdx;

	/* ... */
	return 0.;
}

void ArrowbotSimulator::validateController()
{
	int sensors, motors;
	std::tie(sensors, motors) = currentController->shape();
	if(sensors!=2*segments() || motors!=segments())
	{
		std::cerr << "ANN size (" << sensors << ", " << motors << ") does not match robot size (" << 2*segments() << ", " << segments() << ", exiting\n";
		exit(EXIT_FAILURE);
	}
}

void ArrowbotSimulator::parseController(matrix<double>& W, matrix<double>& Y)
{
	int sensors, motors;
	std::tie(sensors, motors) = currentController->shape();
	W.resize(motors, motors, false);
	Y.resize(motors, motors, false);
	auto ptrWts = currentController->weightsMatrix();
	for(int i=0; i<motors; i++)
		for(int j=0; j<motors; j++)
		{
			W(i,j) = (*ptrWts)[i][j];
			Y(i,j) = (*ptrWts)[motors+i][j];
		}
}

// Public

void ArrowbotSimulator::wire(ANNDirect* newController)
{
	currentController = newController;
	validateCurrentController();
	matrix<double> W,Y,K;
	parseCurrentController(W, Y);
	lowerTriangularOnes(K, segments());
	phiCoefficient = Y - prod(W, prod(parameters.sensorAttachment, K));
	psiCoefficient = W;
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

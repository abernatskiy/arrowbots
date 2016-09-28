#include <tuple>
#include <iostream>
#include <cstdlib>

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/odeint.hpp>

#include "arrowbotSimulator.h"

/* Auxiliary functions */

void exitWithError(const std::string& mess)
{
	std::cerr << mess << std::endl;
	exit(EXIT_FAILURE);
}

void lowerTriangularOnes(matrix<double>& K, int size)
{
	K.resize(size, size);
	for(int i=0; i<size; i++)
		for(int j=0; j<size; j++)
			K(i,j) = i>=j ? 1. : 0.;
}

/* ArrowbotSimulator class definitions */

// Private

void ArrowbotSimulator::validateArrowbotParameters()
{
	if(segments() < 1)
		exitWithError("Bad parameters: Arrowbot must have at least one segment. Exiting");
	if(botParameters.sensorAttachment.size1() != segments() ||
	   botParameters.sensorAttachment.size2() != segments())
		exitWithError("Bad parameters: some size of the sensor attachment table does not match the number of segments. Exiting");
	}
	int maxSensorsPerSegment = 0;
	int sensorsPerSegment = 0;
	for(int i=0; i<segments(); i++)
	{
		for(int j=0; j<segments(); j++)
		{
			if(botParameters.sensorAttachment != 0.)
			{
				if(botParameters.sensorAttachment != 1.)
					exitWithError("Bad parameters: attachment matrix must only contain zeros and ones. Exiting");
				sensorsPerSegment++;
			}
		}
		maxSensorsPerSegment = maxSensorsPerSegment>sensorsPerSegment ? maxSensorsPerSegment : sensorsPerSegment;
	}
	if(maxSensorsPerSegment > 1)
		exitWithError("Bad parameters: having more than one sensor per segment is not supported. Exiting");
}

void ArrowbotSimulator::validateArrowbotSimulationParameters()
{
	if(simParameters.targetOrientations.size() == 0)
		exitWithError("Bad simulation parameters: no target orientations supplied. Exiting");
	if(simParameters.targetOrientations.size() != simParameters.initialConditions.size())
		exitWithError("Bad simulation parameters: initial conditions must be supplied for every target orientation. Exiting");
	if(simParameters.targetOrientations(0).size() != segments() ||
	   simParameters.initialConditions(0).size() != segments())
		exitWithError("Bad simulation parameters: size of target orientation or initial conditions vectors does not match the number of segments. Exiting");
	if(simParameters.totalTime > simParameters.timeStep)
		exitWithError("Bad simulation parameters: time step must be less than or equal to total simulation time. Exiting")
}

void ArrowbotSimulator::validateController()
{
	int sensors, motors;
	std::tie(sensors, motors) = currentController->shape();
	if(sensors!=2*segments() || motors!=segments())
	{
		std:ostringstream os;
		os << "ANN size (" << sensors << ", " << motors << ") does not match robot size (" << 2*segments() << ", " << segments() << ", exiting";
		exitWithError(os.str());
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

double ArrowbotSimulator::evaluateControllerForOrientations(int orientationsIdx)
{
	typedef vector<double> stateType;

	class ArrowbotRHS
	{
		const matrix<double>& m_phiCoefficient;
		stateType psiContrib;

		public:

		ArrowbotRHS(const matrix<double>& phiCoeff, const matrix<double>& psiCoeff, const vector<double>& targetOrientations) :
			m_phiCoefficient(phiCoeff)
		{
			psiContrib = prod(psiCoeff, targetOrientations);
		};
		void operator()(const stateType& x, stateType& dxdt, const double /* t */)
		{
			dxdt = psiContrib;
			axpy_prod(m_phiCoefficient, x, dxdt, false);
		};
	};

	class ArrowbotObserver
	{
		public:

		void operator()(const stateType& x, double t)
		{
			std::cout << "t: " << t << " state: " << x << std::endl;
		}
	};

	using namespace boost::numeric::odeint;

	stateType currentState = simParameters.initialConditions(orientationsIdx);
	runge_kutta4<stateType> stepper;
	ArrowbotRHS abtRHS(phiCoefficient, psiCefficient, simParameters.targetOrientations(orientationsIdx));
	ArrowbotObserver obs;
	integrate_const(stepper, abtRHS, currentState, 0.0, simParameters.totalTime, simParameters.timeStep, obs);

	return 0.;
}

// Public

void ArrowbotSimulator::wire(ANNDirect* newController)
{
	currentController = newController;
	validateController();
	matrix<double> Y, K;
	parseCurrentController(psiCoefficient, Y);
	lowerTriangularOnes(K, segments());
	phiCoefficient = Y - prod(psiCoefficient, prod(botParameters.sensorAttachment, K));
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

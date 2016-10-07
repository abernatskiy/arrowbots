#include <iostream>
#include <tuple>

#include <boost/numeric/ublas/matrix.hpp>

#include "evclib/parseCLI.h"
#include "evclib/evalQueue.h"
#define ANNNodeState double // explicitly reminding ANNDirect which type we want (double is currently the default)
#include "evclib/ann/direct.h"
#include "arrowbotSimulator.h"

#ifndef DM
#define DM if(DEBUG)
#endif // DM

#ifndef DEBUG
#define DEBUG false
#endif // DEBUG

int main(int argc, char** argv)
{
	const int segments = 2;

	// Parsing the command line to get input file names
	std::string inFN, outFN;
	std::tie(inFN, outFN) = parsecli::twoFilenamesForIO(argc, argv, "arrowBotEvaluator");

	// Creating the simulator for the robot
	using namespace boost::numeric::ublas;

	ArrowbotParameters abtParams;
	abtParams.segments = segments;
	abtParams.sensorAttachment = identity_matrix<double>(segments);

	DM std::cout << "Robot parameters:" << std::endl << abtParams << std::endl;

	ArrowbotSimulationParameters abtSimParams;
	abtSimParams.totalTime = 30.;
	abtSimParams.timeStep = 0.1;
	abtSimParams.targetOrientations.resize(1);
	abtSimParams.initialConditions.resize(1);
	abtSimParams.targetOrientations(0).resize(segments);
	abtSimParams.initialConditions(0).resize(segments);
	abtSimParams.targetOrientations(0)(0) = 1.0;
	abtSimParams.targetOrientations(0)(1) = -1.0;
	abtSimParams.initialConditions(0)(0) = 0.8;
	abtSimParams.initialConditions(0)(1) = -0.8;
	abtSimParams.integrateError = false;
	abtSimParams.writeTrajectories = true;

	DM std::cout << "Simulation parameters:" << std::endl << abtSimParams << std::endl;

	ArrowbotSimulator abts(abtParams, abtSimParams);

	// Describing the Arrowbot's controller: two sensors and one motor per segment, identity as transfer function for a purely linear controller
	ANNDirectHyperparameters hyp;
	hyp.inputNodes = 2*segments;
	hyp.outputNodes = segments;
	hyp.transferFunction = [](double x){return x;}; // purely linear controller

	// Creating the evaluation queue and drawing the rest of the owl
	auto evalQueue = EvalQueue<ANNDirect,ANNDirectHyperparameters>(inFN, outFN, hyp);

	while(1)
	{
		auto ptrANN = evalQueue.getNextPhenotypePtr();
		abts.wire(ptrANN);
		abts.evaluateController();
	}

	return 0;
}

#include <iostream>
#include <tuple>

#include <boost/numeric/ublas/matrix.hpp>

#include "evclib/parseCLI.h"
#include "evclib/evalQueue.h"
#define ANNNodeState double // explicitly reminding ANNDirect which type we want (double is currently the default)
#include "evclib/ann/direct.h"
#include "arrowbotSimulator.h"

#define DM if(DEBUG)
#define DEBUG false

int main(int argc, char** argv)
{
	const int segments = 2;

	// Parsing the command line to get input file names
	std::string inFN, outFN;
	std::tie(inFN, outFN) = parsecli::twoFilenamesForIO(argc, argv, "arrowBotEvaluator");

	// Describing the Arrowbot's controller: two sensors and one motor per segment, identity as transfer function for a purely linear controller
	ANNDirectHyperparameters hyp;
	hyp.inputNodes = 2*segments;
	hyp.outputNodes = segments;
	hyp.transferFunction = [](double x){return x;};

	// Creating the simulator for the robot
	using namespace boost::numeric::ublas;

	ArrowbotParameters abtParams;
	abtParams.segments = 2;
	abtParams.sensorAttachment = identity_matrix<double>(abtParams.segments);

	ArrowbotSimulationParameters abtSimParams;
	abtSimParams.totalTime = 1.;
	abtSimParams.timeStep = 0.1;
	abtSimParams.targetOrientations.resize(1);
	abtSimParams.initialConditions.resize(1);
	abtSimParams.targetOrientations(0) = vector<double>(1., -1.);
	abtSimParams.initialConditions(0) = vector<double>(0.1, -0.1);

	ArrowbotSimulator abts(abtParams, abtSimParams);

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

#include <iostream>
#include <tuple>

#include "evclib/parseCLI.h"

#define QUEUE_VERBOSE_EVALUATION
#include "evclib/evalQueue.h"

#define ANNNodeState double // explicitly reminding to the library which type we want (double should be there by default)
#include "evclib/ann/direct.h"

#include "arrowbot.h"

#define DM if(DEBUG)
#define DEBUG false

int main(int argc, char** argv)
{
	const int segments = 3;

	// Parsing the command line to get input file names
	std::string inFN, outFN;
	std::tie(inFN, outFN) = parsecli::twoFilenamesForIO(argc, argv, "arrowBotEvaluator");

	// Describing the Arrowbot's controller: two sensors and one motor per segment, identity as transfer function for a purely linear controller
	ANNDirectHyperparameters hyp;
	hyp.inputNodes = 2*segments;
	hyp.outputNodes = segments;
	hyp.transferFunction = [](double x){return x;};

	// Creating the model of the robot
	ArrowbotSimulator abt(); // likely some arguments will have to go there in the future

	// Creating the evaluation queue and drawing the rest of the owl
	auto evalQueue = EvalQueue<ANNDirect,ANNDirectHyperparameters>(inFN, outFN, hyp);
//	while(1)
	for(int i=0; i<30; i++)
	{
		auto ptrANN = evalQueue.getNextPhenotypePtr();
		abt.wire(ptrANN);
		abt.evaluate();
	}

	return 0;
}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once


namespace ISM {

struct TreeValidatorParameters
{
	unsigned int treeValidatorId = 0;
	unsigned int maxTreeHeight = 0;
};

struct TopologyGeneratorParameters
{
	unsigned int topologyGeneratorId = 0;
	bool swapRelations = true;
	bool removeRelations = true;
	int maxNeighbourCount = -1;
};

struct CostFunctionParameters
{
	unsigned int costFunctionId = 0;
	double alpha = 1;
	double beta = 1;
};

struct OptimizationAlgorithmParameters
{
	unsigned int optimizationAlgorithmId = 0;
	double randomRestartProbability = 0;
	double randomWalkProbability = 0;

	double startTemperature = 0;
	double endTemperature = 0;
	double temperatureFactor = 0;
	unsigned int repetitionsBeforeUpdated = 10;

	double initialAcceptableCostDelta = 0;
	double costDeltaDecreaseFactor = 0;

	bool startFromRandomTopology = false;
};

struct EvaluatorParameters
{
	unsigned int evaluatorId = 0;
	double binSize = 1;
	double maxAngleDeviation = 30;
	double confidenceThreshold = 0.99;
	bool testForFalseNegatives = false;
};


struct CombinatorialTrainerParameters
{
	struct
	{
		std::string dbfilename = "";
		std::string outputDataPath = "";
		std::string loadStartTopologiesFrom = "";

		unsigned int testSetCount = 600;

		std::string loadValidTestSetsFrom = "";
		std::string loadInvalidTestSetsFrom = "";
		std::string storeValidTestSetsTo = "";
		std::string storeInvalidTestSetsTo = "";

		bool useClassifier = false;

		bool storeFullyMeshedISM = false;
		bool storeStartTopologyISM = false;
	} general;

	struct TreeValidatorParameters treeValidator;
	struct TopologyGeneratorParameters topologyGenerator;
	struct OptimizationAlgorithmParameters optimizationAlgorithm;
    struct EvaluatorParameters evaluator;
	struct CostFunctionParameters costFunction;
};

}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "OptimizationRunnerFactory.hpp"

#include "../typedef.hpp"
#include "TreeValidator.hpp"
#include "HeightChecker.hpp"

#include "TopologyGenerator.hpp"
#include "TopologyGeneratorPaper.hpp"
#include "TopologyGeneratorNaive.hpp"

#include "Evaluator.hpp"
#include "Tester.hpp"

#include "utility/LogHelper.hpp"
//#include "utility/TopologyHelper.hpp"
#include "utility/SVGHelper.hpp"

#include "OptimizationRunner.hpp"
#include "../combinatorial_optimization/HillClimbingAlogrithm.hpp"
#include "../combinatorial_optimization/SimulatedAnnealingAlgorithm.hpp"
#include "../combinatorial_optimization/ExponentialCoolingSchedule.hpp"

namespace ISM {

		OptimizationRunnerPtr OptimizationRunnerFactory::createOptimizationRunner(EvaluatorParameters evaluatorParams,
			TopologyGeneratorParameters topologyGeneratorParams,
			OptimizationAlgorithmParameters optimizationAlgorithmParameters,
			CostFunctionParameters costFunctionParameters,
			TreeValidatorParameters treeValidatorParams,
			std::pair<PatternNameToObjectSet, PatternNameToObjectSet> testSets,
			std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern,
			DocumentationHelperPtr documentationHelper,
			bool storeFullyMeshedISM,
			bool storeStartTopologyISM)
	{
		TreeValidatorPtr treeValidator ;

		switch (treeValidatorParams.treeValidatorId)
		{
			case 0:
				treeValidator = TreeValidatorPtr(new HeightChecker(treeValidatorParams.maxTreeHeight));
				break;
			default:
				std::string errorMessage = std::to_string(treeValidatorParams.treeValidatorId)
					+ " is not a valid treeEvaluatorId!";
				LogHelper::logMessage(errorMessage, LOG_ERROR);
				throw std::runtime_error(errorMessage);
		}

		EvaluatorPtr evaluator;
		RecognizerPtr recognizer;
		ObjectSetValidatorPtr objectSetValidator;

		switch (evaluatorParams.evaluatorId)
		{
			case 0:
				recognizer = ISM::RecognizerPtr(new ISM::Recognizer("", evaluatorParams.binSize,
                            evaluatorParams.maxAngleDeviation, false));
				objectSetValidator = ISM::ObjectSetValidatorPtr(new ObjectSetValidator(recognizer,
							evaluatorParams.confidenceThreshold));
				evaluator = EvaluatorPtr(new Tester(objectSetValidator, testSets.first, testSets.second,
													evaluatorParams.testForFalseNegatives));
				break;
			default:
				std::string errorMessage = std::to_string(evaluatorParams.evaluatorId) + " is not a valid evaluatorId!";
				LogHelper::logMessage(errorMessage, LOG_ERROR);
				throw std::runtime_error(errorMessage);
		}


		TopologyGeneratorPtr topologyGenerator;

		switch (topologyGeneratorParams.topologyGeneratorId)
		{
			case 0 :
				topologyGenerator = TopologyGeneratorPaperPtr(new TopologyGeneratorPaper(allObjectRelationsPerPattern,
							topologyGeneratorParams.maxNeighbourCount,
							topologyGeneratorParams.swapRelations,
							topologyGeneratorParams.removeRelations));
				break;
			case 1:
				topologyGenerator = TopologyGeneratorNaivePtr(new TopologyGeneratorNaive(allObjectRelationsPerPattern,
							topologyGeneratorParams.maxNeighbourCount));
				break;
			default :
				std::string errorMessage = std::to_string(topologyGeneratorParams.topologyGeneratorId) +
					" is not a valid successorGeneratorId!";
				LogHelper::logMessage(errorMessage, LOG_ERROR);
				throw std::runtime_error(errorMessage);
		}

		TopologyManagerPtr topologyManager = TopologyManagerPtr(new TopologyManager(evaluator, treeValidator,
					topologyGenerator, documentationHelper));

		return OptimizationRunnerPtr(new OptimizationRunner(
					topologyManager, documentationHelper,
					optimizationAlgorithmParameters, costFunctionParameters,
					storeFullyMeshedISM, storeStartTopologyISM
				));

	}

}


/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "OptimizationRunner.hpp"

#include "CombinatorialTrainerParameters.hpp"
#include "WeightedSum.hpp"

#include "../combinatorial_optimization/HillClimbingAlogrithm.hpp"

#include "../combinatorial_optimization/ExponentialCoolingSchedule.hpp"
#include "../combinatorial_optimization/SimulatedAnnealingAlgorithm.hpp"

#include "../combinatorial_optimization/RecordHuntAlgorithm.hpp"
#include "../combinatorial_optimization/CostDeltaAcceptanceFunction.hpp"

namespace ISM {

	std::pair<double, TreePtr> OptimizationRunner::runOptimization(const std::string& pattern,
			TopologyPtr startTopology)
	{
		mStartTime = ptime(boost::posix_time::microsec_clock::local_time());
		mCurrentPatternName = pattern;

		logOptimizationStart(pattern);

		std::vector<TopologyPtr> startTopologies;
		if (startTopology)
		{
			startTopologies.push_back(startTopology);
		}

		prepareOptimizationRun(startTopologies);
		TopologyPtr selectedStartTopology = selectStartTopology(startTopologies);


		if (!selectedStartTopology)
		{
			//There was no valid start topology, optimization failed
			return std::make_pair(-1, TreePtr());
		}

		TopologyPtr bestTopology = mOptimizationAlgorithm->optimize(selectedStartTopology);

		//Check if hill climbing is used for the optimization
		if (mOptimizationAlgorithmParameters.optimizationAlgorithmId == 0)
		{
			//Check for random restart should be performed 
			while (mDistribution(mGenerator) < mOptimizationAlgorithmParameters.randomRestartProbability)
			{
				logRandomRestart();
				TopologyPtr randomStartTopology = mTopologyManager->getRandomTopology();
				TopologyPtr optimizationResult = mOptimizationAlgorithm->optimize(randomStartTopology);

				startTopologies.clear();
				startTopologies.push_back(randomStartTopology);
				mTopologyManager->addStartTopologiesToHistory(startTopologies);

				if (optimizationResult->cost < bestTopology->cost)
				{
					bestTopology = optimizationResult;
				}
			}
		}

		logOptimizationFinish(bestTopology);
		documentOptimzationRun(bestTopology);

		TreePtr tree = TreePtr(new Tree(mCurrentPatternName, bestTopology->objectRelations));

		mDocumentationHelper->storeTopology(bestTopology, mCurrentPatternName, "best_topology",
				tree->getISM()->voteSpecifiersPerObject);

		return std::make_pair(mCostFunction->calculateCost(bestTopology), tree);
	}

	void OptimizationRunner::prepareOptimizationRun(std::vector<TopologyPtr>& startTopologies)
	{
		mTopologyManager->setUp(mCurrentPatternName);

		analyzeStarAndFullyMeshedTopologies();
		initCostFunction();
		initOptimizationAlgorithm();

		if (startTopologies.empty())
		{
			if (mOptimizationAlgorithmParameters.startFromRandomTopology)
			{
				TopologyPtr randomTopology;
				while(true)
				{
					randomTopology = mTopologyManager->getRandomTopology();
					if (mCostFunction->calculateCost(randomTopology) != std::numeric_limits<double>::infinity())
					{
						break;
					}
				}

				startTopologies.push_back(randomTopology);
			}
			else
			{
				//We have no specific topologies from wich we want to start the optimization,
				//so we will use the star topologies as our initial topologies.
				startTopologies = mStarTopologies;
			}
		}
		else
		{
			for (unsigned int i = 0; i < startTopologies.size(); ++i)
			{
				mTopologyManager->evaluateTopology(startTopologies[i],  "start_topology_" + std::to_string(i));
			}
		}

		mTopologyManager->addStartTopologiesToHistory(startTopologies);
	}

	void OptimizationRunner::analyzeStarAndFullyMeshedTopologies()
	{
		logAnalysingStarAndFullyMeshedTopologies();

		mStarTopologies = mTopologyManager->getStarTopologies();
		mFullyMeshedTopology = mTopologyManager->getFullyMeshedTopology(mStoreFullyMeshedISM);

		//We normalise the number of false positives between 0 (the amount of false positives
		//of the fully meshed topology) and the highest amount of false positves of all starTopologies.
		//The recogniton runtime is normalised between the fastest runtime which can be found among
		//the star topologies and the runtime of the fully meshed topology.

		mMaxFalsePositives = std::numeric_limits<unsigned int>::min();
		mMinAverageRecognitionRuntime = std::numeric_limits<double>::max();

		for (TopologyPtr starTopology : mStarTopologies)
		{
			EvaluationResult er = starTopology->evaluationResult;

			mMinAverageRecognitionRuntime = std::min(er.averageRecognitionRuntime, mMinAverageRecognitionRuntime);
			mMaxFalsePositives = std::max(er.falsePositives, mMaxFalsePositives);
		}

		mMinFalsePositives = mFullyMeshedTopology->evaluationResult.falsePositives;
		mMaxAverageRecognitionRuntime = mFullyMeshedTopology->evaluationResult.averageRecognitionRuntime;

		//The number of false positives of the fully meshed topology must be 0,
		//because we used it to classify the testsets.
		assert(mMinFalsePositives == 0);

		logStarsAndFullyMeshedResult();

		//Add an epsilon to mMaxAverageRecognitionRuntime to account for uncertainties in the runtime measurement.
		double epsilon = 0.05 * mMaxAverageRecognitionRuntime;
		mMaxAverageRecognitionRuntime += epsilon;
	}

	void OptimizationRunner::initCostFunction()
	{
		switch (mCostFunctionParameters.costFunctionId)
		{
			case 0:
				{
					mCostFunction = CostFunctionPtr<TopologyPtr>(new WeightedSum(mMinFalsePositives, mMaxFalsePositives,
								mMinAverageRecognitionRuntime, mMaxAverageRecognitionRuntime,
								mCostFunctionParameters.alpha, mCostFunctionParameters.beta));
				}
				break;
			default:
				std::string errorMessage = std::to_string(mCostFunctionParameters.costFunctionId)
					+ " is not a valid costFunctionId!";
				LogHelper::logMessage(errorMessage, LOG_ERROR);
				throw std::runtime_error(errorMessage);
		}
	}

	void OptimizationRunner::initOptimizationAlgorithm()
	{
		switch (mOptimizationAlgorithmParameters.optimizationAlgorithmId)
		{
			case 0:
				mOptimizationAlgorithm = HillClimbingAlogrithmPtr<TopologyPtr>(
						new HillClimbingAlogrithm<TopologyPtr>(
							mTopologyManager,
							mCostFunction,
							mOptimizationAlgorithmParameters.randomWalkProbability
							));
				break;
			case 1:
				{
					CoolingSchedulePtr coolingSchedule = ExponentialCoolingSchedulePtr(
							new ExponentialCoolingSchedule(
								mOptimizationAlgorithmParameters.startTemperature,
								mOptimizationAlgorithmParameters.endTemperature,
								mOptimizationAlgorithmParameters.repetitionsBeforeUpdated,
								mOptimizationAlgorithmParameters.temperatureFactor
								));

					mOptimizationAlgorithm = SimulatedAnnealingAlgorithmPtr<TopologyPtr>(
							new SimulatedAnnealingAlgorithm<TopologyPtr>(
								mTopologyManager, mCostFunction, coolingSchedule
								));
				}
				break;
			case 2:
				{
					AcceptanceFunctionPtr acceptanceFunction = CostDeltaAcceptanceFunctionPtr(
							new CostDeltaAcceptanceFunction(
								mOptimizationAlgorithmParameters.initialAcceptableCostDelta,
								mOptimizationAlgorithmParameters.costDeltaDecreaseFactor
								));

					mOptimizationAlgorithm = RecordHuntAlgorithmPtr<TopologyPtr>(
							new RecordHuntAlgorithm<TopologyPtr>(
								mTopologyManager, mCostFunction, acceptanceFunction
								));
				}
				break;
			default:
				std::string errorMessage = std::to_string(mOptimizationAlgorithmParameters.optimizationAlgorithmId)
					+ " is not a valid optimizationAlgorithmId!";
				LogHelper::logMessage(errorMessage, LOG_ERROR);
				throw std::runtime_error(errorMessage);
		}
	}

	TopologyPtr OptimizationRunner::selectStartTopology(std::vector<TopologyPtr>& startTopologies)
	{
		logStartTopologies(startTopologies);
		TopologyPtr bestStartTopology;
		double bestCost = std::numeric_limits<unsigned int>::max();

		for (TopologyPtr startTopology : startTopologies)
		{
			double cost = mCostFunction->calculateCost(startTopology);

			if (cost < bestCost)
			{
				bestCost = cost;
				bestStartTopology = startTopology;
			}
		}

		if (!bestStartTopology)
		{
			return bestStartTopology;
		}

		logSelectedStartTopology(bestStartTopology);
		TreePtr tree = TreePtr(new Tree(mCurrentPatternName, bestStartTopology->objectRelations));

		if (mStoreStartTopologyISM)
		{
			mDocumentationHelper->storeIsm((mCurrentPatternName + "_start"), tree->getISM());
		}


		mDocumentationHelper->storeTopology(bestStartTopology, mCurrentPatternName, "selected_start_topology",
				tree->getISM()->voteSpecifiersPerObject);

		LogHelper::logLine();

		return bestStartTopology;
	}

	void OptimizationRunner::documentOptimzationRun(TopologyPtr bestTopology)
	{
		ptime endTime(boost::posix_time::microsec_clock::local_time());
		time_duration td = endTime - mStartTime;

		std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>> history = mTopologyManager->getHistory();

		//Mark the best topology in the history
		const std::string bestTopologyIdentifier = bestTopology->identifier;
		bool markedBest = false;
		for (unsigned int i = 0; !markedBest && i < history.size(); i++)
		{
			for (unsigned int j = 0; !markedBest && j < history[i].size(); j++)
			{
				if (history[i][j].first->identifier == bestTopologyIdentifier)
				{
					history[i][j].second = 4;
					markedBest = true;
				}
			}
		}

		mDocumentationHelper->storeOptimizationRun(history, mCostFunction, td.total_seconds(),
				mCurrentPatternName);
	}

	void OptimizationRunner::logOptimizationStart(const std::string& patternName)
	{
		LogHelper::logMessage("Started combinatorial optimization to find the best the ism for the pattern " +
				patternName, LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
		LogHelper::logLine();
	}

	void OptimizationRunner::logOptimizationFinish(TopologyPtr bestTopology)
	{
		std::ostringstream oss;
		oss <<"Training for pattern " + mCurrentPatternName + " is done!" << std::endl << std::endl
			<<"The overall best topology was:  " << bestTopology->objectRelations << std::endl
			<< " with the evaluation result of its ISM beeing: " << std::endl
			<< bestTopology->evaluationResult.getDescription() << std::endl << std::endl
			<< "The cost of this topology was: " << std::endl
			<< mCostFunction->calculateCost(bestTopology);
		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
		LogHelper::logLine();
	}

	void OptimizationRunner::logSelectedStartTopology(TopologyPtr bestStartTopology)
	{
		std::ostringstream oss;
		oss << "Selected the following topology as the best start Topology."
			<< std::endl << std::endl << bestStartTopology->objectRelations
			<< "with the evaluation result of its ISM beeing: " << std::endl
			<< bestStartTopology->evaluationResult.getDescription() << std::endl << std::endl
			<< "The cost of this topology is: " << std::endl
			<< mCostFunction->calculateCost(bestStartTopology) << std::endl << std::endl
			<< "Generating its tree again: " << std::endl;
		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
	}

	void OptimizationRunner::logAnalysingStarAndFullyMeshedTopologies()
	{
		std::ostringstream oss;
		oss << "Evaluating all star topologies and the fully meshed topology "
			<< "to find the lowest and highest false positives and the slowest and fastest average recogniton runtime. "
			<< "Those values will be used to normalise the false positives and the recognition runtime of other topologies."
			<< std::endl << std::endl;
		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
	}

	void OptimizationRunner::logStarsAndFullyMeshedResult()
	{
		std::ostringstream oss;
		oss << "Evaluated all star topologies and the fully meshed topology. The results are:" << std::endl << std::endl
			<< "\t Lowest false positives: " << mMinFalsePositives << std::endl
			<< "\t Highest false positives: " << mMaxFalsePositives << std::endl
			<< "\t Shortest average recognition runtime: " << mMinAverageRecognitionRuntime << "s" << std::endl
			<< "\t Longest average recognition runtime: " << mMaxAverageRecognitionRuntime << "s" << std::endl;
		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
		LogHelper::logLine();
	}

	void OptimizationRunner::logInvalidStartTopology()
	{
		std::ostringstream oss;
		oss << "The start topology was invalid. Optimzation Failed!";
		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
		LogHelper::logLine();
	}

	void OptimizationRunner::logRandomRestart()
	{
		LogHelper::logLine();
		std::ostringstream oss;
		oss << "Performing random restart!" << std::endl;
		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
	}

	void OptimizationRunner::logStartTopologies(const std::vector<TopologyPtr> & startTopologies)
	{
		std::ostringstream oss;
		oss << "Possible start topologies are: " << std::endl << std::endl;
		for (unsigned int i = 0; i < startTopologies.size(); ++i)
		{
			oss <<  "\t- Topology " << startTopologies[i]->identifier << std::endl;
		}

		LogHelper::logMessage(oss.str(), LOG_INFO, LOG_COLOR_OPTIMIZATION_STRATEGY);
		LogHelper::logLine();
	}


}

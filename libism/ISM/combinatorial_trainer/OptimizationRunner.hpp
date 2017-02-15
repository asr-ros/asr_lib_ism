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

#include <boost/filesystem/path.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "Tree.hpp"
#include "Evaluator.hpp"
#include "TreeValidator.hpp"
#include "ObjectRelation.hpp"
#include "CombinatorialTrainerParameters.hpp"
#include "TopologyGenerator.hpp"
#include "Topology.hpp"
#include "TopologyManager.hpp"

#include "../combinatorial_optimization/OptimizationAlgorithm.hpp"
#include "../combinatorial_optimization/CostFunction.hpp"

#include "utility/LogHelper.hpp"
#include "utility/DocumentationHelper.hpp"

#include <ctime>

namespace ISM {

using boost::posix_time::ptime;
using boost::posix_time::time_duration;

class OptimizationRunner
{
	public:

		OptimizationRunner(TopologyManagerPtr topologyManager,
				DocumentationHelperPtr documentationHelper,
				OptimizationAlgorithmParameters optimizationAlgorithmParameters,
				CostFunctionParameters costFunctionParameters,
				bool storeFullyMeshedISM, bool storeStartTopologyISM)
			: mTopologyManager(topologyManager)
			, mDocumentationHelper(documentationHelper)
			, mOptimizationAlgorithmParameters(optimizationAlgorithmParameters)
			, mCostFunctionParameters(costFunctionParameters)
			, mStoreFullyMeshedISM(storeFullyMeshedISM)
			, mStoreStartTopologyISM(storeStartTopologyISM)
		{}

		std::pair<double, TreePtr> runOptimization(const std::string& pattern, TopologyPtr startTopology = NULL);

	private:

		TopologyManagerPtr mTopologyManager;
		DocumentationHelperPtr mDocumentationHelper;
		CostFunctionPtr<TopologyPtr> mCostFunction;
		OptimizationAlgorithmPtr<TopologyPtr> mOptimizationAlgorithm;

		OptimizationAlgorithmParameters mOptimizationAlgorithmParameters;
		CostFunctionParameters mCostFunctionParameters;

		std::default_random_engine mGenerator;
		std::uniform_real_distribution<double> mDistribution = std::uniform_real_distribution<double>(0.0, 1.0);

		std::string mCurrentPatternName;
		ptime mStartTime;

		bool mStoreFullyMeshedISM;
		bool mStoreStartTopologyISM;

		std::vector<TopologyPtr> mStarTopologies;
		TopologyPtr mFullyMeshedTopology;

		unsigned int mMinFalsePositives;
		unsigned int mMaxFalsePositives;

		double mMinAverageRecognitionRuntime;
		double mMaxAverageRecognitionRuntime;

		void prepareOptimizationRun(std::vector<TopologyPtr>& startTopologies);
		void analyzeStarAndFullyMeshedTopologies();
		TopologyPtr selectStartTopology(std::vector<TopologyPtr>& startTopologies);

		void documentOptimzationRun(TopologyPtr bestTopology);
		void initCostFunction();
		void initOptimizationAlgorithm();

		void logOptimizationStart(const std::string& patternName);
		void logOptimizationFinish(TopologyPtr bestTopology);

		void logAnalysingStarAndFullyMeshedTopologies();
		void logStarsAndFullyMeshedResult();

		void logWeightedSum();
		void logStartTopologies(const std::vector<TopologyPtr> & startTopologies);
		void logSelectedStartTopology(TopologyPtr bestStartTopology);
		void logRandomRestart();
		void logInvalidStartTopology();

		const char* LOG_COLOR_OPTIMIZATION_STRATEGY = LogHelper::LOG_COLOR_DEFAULT;

}; typedef boost::shared_ptr<OptimizationRunner> OptimizationRunnerPtr;

}




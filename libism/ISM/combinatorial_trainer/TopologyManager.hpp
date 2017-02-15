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

#include <boost/shared_ptr.hpp>
#include "map"

#include "ObjectRelation.hpp"
#include "utility/LogHelper.hpp"
#include "utility/DocumentationHelper.hpp"

#include "ConnectivityChecker.hpp"
#include "Topology.hpp"

#include "Evaluator.hpp"
#include "TreeValidator.hpp"
#include "TopologyGenerator.hpp"
#include "ObjectRelation.hpp"
#include "../combinatorial_optimization/NeighbourhoodFunction.hpp"

namespace ISM {

class TopologyManager : public NeighbourhoodFunction<TopologyPtr>
{
	public:
		TopologyManager(EvaluatorPtr evaluator, TreeValidatorPtr treeValidator,
				TopologyGeneratorPtr topologyGenerator, DocumentationHelperPtr documentationHelper)
			: mEvaluator(evaluator)
			, mTreeValidator(treeValidator)
			, mTopologyGenerator(topologyGenerator)
			, mDocumentationHelper(documentationHelper)
		{}

		TopologyPtr getNextNeighbour();
		bool hasNextNeighbour();
		void setReferenceInstance(TopologyPtr instance);

		std::vector<TopologyPtr> prepareStartTopologies(std::vector<TopologyPtr>& startTopologies);
		void setUp(const std::string & patternName);

		std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>> getHistory();

		TopologyPtr getFullyMeshedTopology(bool storeFullyMeshedISM = false);
		std::vector<TopologyPtr> getStarTopologies();

		void addStartTopologiesToHistory(std::vector<TopologyPtr> startTopologies);
		TreePtr evaluateTopology(TopologyPtr& topology, const std::string filename = "");

		TopologyPtr getRandomTopology();

	private:
		EvaluatorPtr mEvaluator;
		TreeValidatorPtr mTreeValidator;
		TopologyGeneratorPtr mTopologyGenerator;
		DocumentationHelperPtr mDocumentationHelper;

		bool mHasNextInstance;

		std::map<std::string, std::map<std::string, TopologyPtr>> mPatternToTopologyIdToTopolgy;
		std::string mCurrentPatternName;

		std::vector<TopologyPtr> mNeighbourTopologies;
		unsigned mNeighbourIndex;

		std::map<std::string, unsigned int> mPatternToTopologyIndexCounter;

		int mHistoryIndex = 0;
		std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>> mHistory;
		unsigned int mNumTopology = 0;

		void prepareTopology(TopologyPtr& topology);

		void calculateNeighbours(TopologyPtr topology);

		void markSelectedTopology(TopologyPtr selectedTopology);

		//Console output
		void logAlreadyAnalysed(TopologyPtr topology);
		void logSelectedTopology(TopologyPtr selectedTopology);

}; typedef boost::shared_ptr<TopologyManager> TopologyManagerPtr;

}

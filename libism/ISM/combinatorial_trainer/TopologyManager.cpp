/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "TopologyManager.hpp"

namespace ISM {

	TopologyPtr TopologyManager::getNextNeighbour()
	{
		return mNeighbourTopologies[mNeighbourIndex++];
	}

	TopologyPtr TopologyManager::getRandomTopology()
	{

		std::ostringstream oss;
		oss << "Generating random Topologies until a valid one is found:" << std::endl;
		LogHelper::logMessage(oss.str(), LOG_INFO);

		TopologyPtr randomTopology = mTopologyGenerator->generateRandomTopology(mCurrentPatternName);
		oss.str("");
		oss << std::endl << std::endl << randomTopology->objectRelations;
		LogHelper::logMessage(oss.str(), LOG_INFO);
		evaluateTopology(randomTopology);
		bool foundValidTopology = randomTopology->isValid;

		while(!foundValidTopology)
		{
			oss.str("");
			oss << "Random Topology was invalid, generating a new one." << std::endl;
			LogHelper::logMessage(oss.str(), LOG_INFO);
			randomTopology = mTopologyGenerator->generateRandomTopology(mCurrentPatternName);
			oss.str("");
			oss << std::endl << std::endl << randomTopology->objectRelations;
			LogHelper::logMessage(oss.str(), LOG_INFO);

			evaluateTopology(randomTopology);

			foundValidTopology = randomTopology->isValid;
		}

		LogHelper::logLine();

		return randomTopology;
	}

	bool TopologyManager::hasNextNeighbour()
	{
		while (mNeighbourIndex < mNeighbourTopologies.size())
		{
			TopologyPtr topology = mNeighbourTopologies[mNeighbourIndex];
			prepareTopology(topology);

			if (topology->isValid)
			{
				return true;
			}
			mNeighbourIndex++;
		}

		return false;
	}

	void TopologyManager::setReferenceInstance(TopologyPtr instance)
	{
		markSelectedTopology(instance);
		logSelectedTopology(instance);
		mDocumentationHelper->setReferenceTopology(instance, mCurrentPatternName);
		mHistory.push_back(std::vector<std::pair<TopologyPtr, unsigned int>>());
		mHistoryIndex++;

		mNeighbourIndex = 0;
		mNeighbourTopologies = mTopologyGenerator->generateNeighbours(mCurrentPatternName, instance);
	}

	std::vector<TopologyPtr> TopologyManager::prepareStartTopologies(std::vector<TopologyPtr>& startTopologies)
	{
		std::vector<TopologyPtr> validStartTopologies;
		for (TopologyPtr topology : startTopologies)
		{
			prepareTopology(topology);
			if (topology->isValid)
			{
				validStartTopologies.push_back(topology);
			}
		}

		return validStartTopologies;
	}

	void TopologyManager::setUp(const std::string & patternName)
	{
		mCurrentPatternName = patternName;
		mNumTopology = 1;

		mHistory.clear();
		mHistory.push_back(std::vector<std::pair<TopologyPtr, unsigned int>>());
		mHistoryIndex = 0;

		if (mPatternToTopologyIdToTopolgy.find(mCurrentPatternName) == mPatternToTopologyIdToTopolgy.end())
		{
			mPatternToTopologyIdToTopolgy[mCurrentPatternName] = std::map<std::string, TopologyPtr>();
		}
	}

	std::vector<std::vector<std::pair<TopologyPtr, unsigned int>>> TopologyManager::getHistory()
	{
		return mHistory;
	}

	TopologyPtr TopologyManager::getFullyMeshedTopology(bool storeFullyMeshedISM)
	{
		TopologyPtr fullyMeshed = mTopologyGenerator->generateFullyMeshedTopology(mCurrentPatternName);
		std::stringstream oss;
		oss << "Analysing the fully meshed topology: " << std::endl << std::endl
			<< fullyMeshed->objectRelations;
		LogHelper::logMessage(oss.str(), LOG_INFO);
		TreePtr tree = evaluateTopology(fullyMeshed, "fully_meshed_topology");
		LogHelper::logLine();

		if (storeFullyMeshedISM)
		{
			mDocumentationHelper->storeIsm((mCurrentPatternName + "_fully_meshed"), tree->getISM());
		}

		return fullyMeshed;
	}

	void TopologyManager::addStartTopologiesToHistory(std::vector<TopologyPtr> startTopologies)
	{
		for (TopologyPtr startTopology : startTopologies)
		{
			startTopology->index = mPatternToTopologyIndexCounter[mCurrentPatternName]++;
			mHistory[mHistoryIndex].push_back(std::make_pair(startTopology, 0));

		}
	}

	std::vector<TopologyPtr> TopologyManager::getStarTopologies()
	{
		std::ostringstream oss;
		std::vector<TopologyPtr> starTopologies = mTopologyGenerator->generateStarTopologies(mCurrentPatternName);

		for (unsigned int i = 0; i < starTopologies.size(); ++i)
		{
			TopologyPtr starTopology = starTopologies[i];

			oss << "Analysing star topology " <<  (i + 1) << ": " << std::endl << std::endl
				<< starTopology->objectRelations;
			LogHelper::logMessage(oss.str(), LOG_INFO);

			evaluateTopology(starTopology,  "star_topology_" + std::to_string(i));

			oss.str("");
			LogHelper::logLine();
		}

		return starTopologies;
	}

	void TopologyManager::markSelectedTopology(TopologyPtr selectedTopology)
	{
		for (unsigned int i = 0; i < mHistory[mHistoryIndex].size(); ++i)
		{
			if (mHistory[mHistoryIndex][i].first->identifier == selectedTopology->identifier)
			{
				mHistory[mHistoryIndex][i].second = 1;
				break;
			}
		}
	}

	TreePtr TopologyManager::evaluateTopology(TopologyPtr& topology, const std::string filename)
	{
		TreePtr tree = TreePtr(new Tree(mCurrentPatternName, topology->objectRelations));

		if (mTreeValidator->isTreeValid(tree))
		{
			EvaluationResult er = mEvaluator->evaluate(mCurrentPatternName, tree->getISM());

			topology->evaluationResult = er;
			topology->isValid = true;

			if (!filename.empty())
			{
				mDocumentationHelper->storeTopology(topology, mCurrentPatternName, filename,
						tree->getISM()->voteSpecifiersPerObject);
			}
		}
		else
		{
			topology->isValid = false;
		}

		mPatternToTopologyIdToTopolgy[mCurrentPatternName][topology->identifier] = topology;
		return tree;
	}

	void TopologyManager::prepareTopology(TopologyPtr& topology)
	{
		std::ostringstream oss;
		oss << "Now analysing the following relation topology (Topology number "
			<<  mNumTopology << ") : " << std::endl << std::endl << topology->objectRelations;
		LogHelper::logMessage(oss.str(), LOG_INFO);

		const std::string identifier = topology->identifier;

		if (mPatternToTopologyIdToTopolgy[mCurrentPatternName].find(identifier) ==
				mPatternToTopologyIdToTopolgy[mCurrentPatternName].end())
		{
			evaluateTopology(topology, "topology_" + std::to_string(mPatternToTopologyIndexCounter[mCurrentPatternName]));

			if (topology->isValid)
			{
				topology->index = mPatternToTopologyIndexCounter[mCurrentPatternName]++;
			}
			mNumTopology++;
		}
		else
		{
			topology = mPatternToTopologyIdToTopolgy[mCurrentPatternName][identifier];
			logAlreadyAnalysed(topology);
		}

		if (topology->isValid)
		{
			mHistory[mHistoryIndex].push_back(std::make_pair(topology, 0));
		}

		LogHelper::logLine();
	}

	void TopologyManager::logAlreadyAnalysed(TopologyPtr topology)
	{
		std::ostringstream oss;
		oss << "Already analyzed this topology before!" << std::endl << std::endl;
		if (topology->isValid)
		{
			oss << "The corresponding tree was valid and the evaluation result of its ISM was: " << std::endl
				<< topology->evaluationResult.getDescription();
		}
		else
		{
			oss << " The corresponding tree was invalid!";
			oss << std::endl;
		}
		LogHelper::logMessage(oss.str(), LOG_INFO);
	}

	void TopologyManager::logSelectedTopology(TopologyPtr selectedTopology)
	{
		std::ostringstream oss;
		oss << "Selected the following topology : "
			<< std::endl << std::endl << selectedTopology->objectRelations
			<< "with the evaluation result of its ISM beeing: " << std::endl
			<< selectedTopology->evaluationResult.getDescription() << std::endl << std::endl
			<< "The cost of this topology is: " << std::endl
			<< selectedTopology->cost;
		LogHelper::logMessage(oss.str(), LOG_INFO);
		LogHelper::logLine();
	}
}



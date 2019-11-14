/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "TopologyGenerator.hpp"
#include "../utility/LogHelper.hpp"
#include <random>

namespace ISM {

	TopologyGenerator::TopologyGenerator(const std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern,
				int maxNeighbourCount)
		: mAllObjectRelationsPerPattern(allObjectRelationsPerPattern)
		, mMaxNeighbourCount(maxNeighbourCount)
	{
		std::map<std::string, unsigned int> numObjectsPerPattern;

		for (std::map<std::string, ISM::ObjectRelations>::iterator it = mAllObjectRelationsPerPattern.begin();
				it != mAllObjectRelationsPerPattern.end();
				++it)
		{
			unsigned int numRelations = it->second.size();
			unsigned int numObjects = ((std::sqrt(8 * numRelations + 1) - 1) / 2) + 1;

			mNumRelationsPerPattern[it->first] = numRelations;
			numObjectsPerPattern[it->first] = numObjects;
		}

		mConnectivityChecker = ConnectivityCheckerPtr(new ConnectivityChecker(numObjectsPerPattern));
	}

	std::vector<TopologyPtr> TopologyGenerator::generateNeighbours(const std::string &pattern, TopologyPtr from)
	{
		std::vector<std::vector<bool>> neighbours = calculateNeighbours(pattern, convertTopologyToBitvector(from, pattern));
		std::vector<std::vector<bool>> filteredNeighours = filterBitvectors(neighbours, pattern);
		std::vector<std::vector<bool>> selectedNeighbours = selectNeighbours(filteredNeighours);
		std::vector<TopologyPtr> topologies = convertBitvectors(selectedNeighbours, pattern);
		logNeighbourGeneration(topologies, from, filteredNeighours.size());
		return topologies;
	}

	std::vector<TopologyPtr> TopologyGenerator::generateStarTopologies(const std::string &pattern) {
		std::vector<std::vector<bool>> starBitVectors;

		unsigned int numAllRelations = mNumRelationsPerPattern[pattern];
		unsigned int numObjects = ((std::sqrt(8 * numAllRelations + 1) - 1) / 2) + 1;

		for (unsigned int i = 0; i < numObjects; ++i)
		{
			std::vector<bool> star(numAllRelations, 0);
			unsigned int pos = 0;
			for (unsigned int j = 0; j < i; ++j) {
				star[pos + i - (j + 1)] = 1;
				pos += numObjects - (j + 1);
			}
			for (unsigned int k = pos; k < pos + numObjects - 1 - i; ++k) {
				star[k] = 1;
			}
			starBitVectors.push_back(star);
		}

		return convertBitvectors(starBitVectors, pattern);
	}

	TopologyPtr TopologyGenerator::generateFullyMeshedTopology(const std::string &pattern)
	{
		unsigned int numRelations = mAllObjectRelationsPerPattern[pattern].size();
		std::vector<bool> fullyMeshed(numRelations, true);

		return convertBitvectorToTopology(fullyMeshed, pattern);
	}

	TopologyPtr TopologyGenerator::generateRandomTopology(const std::string &pattern)
	{
		unsigned int numAllRelations = mAllObjectRelationsPerPattern[pattern].size();

		std::random_device rd;
		std::mt19937 eng(rd());
		std::uniform_int_distribution<unsigned int> dist(0, 1);

		std::vector<bool> relations(numAllRelations, 0);
		do
		{
			for (unsigned int i = 0; i < numAllRelations; i++)
			{
				relations[i] = dist(eng);
			}
		}
		while (!mConnectivityChecker->isConnected(relations, pattern));

		return convertBitvectorToTopology(relations, pattern);
	}


	std::vector<std::vector<bool>> TopologyGenerator::selectNeighbours(std::vector<std::vector<bool>>& from)
	{
		if (mMaxNeighbourCount < 0 || mMaxNeighbourCount >= (int) from.size())
			return from;

		//Sort from by #relations.
		struct compare {
			bool operator() (const std::vector<bool> & first, const std::vector<bool> & second) {
				return first.size() > second.size();
			}
		};

		std::sort(from.begin(), from.end(), compare());

		std::random_device rd;
		std::mt19937 eng(rd());
		std::normal_distribution<double> dist(0, from.size() / 2);

		std::vector<std::vector<bool>> selectedNeighbours;

		for (unsigned int i = 0; i < (unsigned int) mMaxNeighbourCount; ++i)
		{
			unsigned int randIndex;
			do {
				randIndex = std::abs(dist(eng));
			} while (randIndex >= from.size());

			selectedNeighbours.push_back(from[randIndex]);
			from.erase(from.begin() + randIndex);

		}

		return selectedNeighbours;
	}

	TopologyPtr TopologyGenerator::convertBitvectorToTopology(const std::vector<bool> & bitvector,
			const std::string& pattern)
	{
		ObjectRelations objectRelations;

		std::ostringstream oss;
		std::vector<unsigned int> relationIndices;

		for (unsigned int i = 0; i < bitvector.size(); ++i)
		{
			if (bitvector[i])
			{
				relationIndices.push_back(i);
				objectRelations[i] = mAllObjectRelationsPerPattern[pattern][i];
			}
		}

		oss << "[" << relationIndices[0];
		for (unsigned int i = 1; i < relationIndices.size(); ++i)
		{
			oss << ", " << relationIndices[i];
		}
		oss << "]";

		TopologyPtr topology = TopologyPtr(new Topology());
		topology->objectRelations = objectRelations;
		topology->identifier = oss.str();
		return topology;
	}

	std::vector<bool> TopologyGenerator::convertTopologyToBitvector(TopologyPtr topology,
			const std::string& pattern)
	{
		unsigned int numAllRelations = mNumRelationsPerPattern[pattern];
		std::vector<bool> bitvector(numAllRelations, 0);
		ObjectRelations objectRelations = topology->objectRelations;
		for (ObjectRelations::iterator it = objectRelations.begin(); it != objectRelations.end(); ++it)
		{
			bitvector[it->first] = 1;
		}

		return bitvector;
	}


	std::vector<std::vector<bool>> TopologyGenerator::filterBitvectors(std::vector<std::vector<bool>> bitvectors,
			const std::string& pattern)
	{
		std::vector<std::vector<bool>> result;
		for (unsigned int i = 0; i < bitvectors.size(); ++i)
		{
			if (mConnectivityChecker->isConnected(bitvectors[i], pattern))
			{
				result.push_back(bitvectors[i]);
			}
		}

		return result;
	}

	std::vector<TopologyPtr> TopologyGenerator::convertBitvectors(std::vector<std::vector<bool>> bitvectors,
			const std::string& pattern)
	{
		std::vector<TopologyPtr> result;
		for (unsigned int i = 0; i < bitvectors.size(); ++i)
		{
			result.push_back(convertBitvectorToTopology(bitvectors[i], pattern));
		}

		return result;
	}

	void TopologyGenerator::logNeighbourGeneration(const std::vector<TopologyPtr> & neighbours,
			TopologyPtr from, const unsigned int totalNumber)
	{
		std::ostringstream oss;

		if (neighbours.size() == 0)
		{
			oss << from->objectRelations << "does not have any neighbours"
				<< std::endl << std::endl;
		} else {
			oss << "Generated " << totalNumber << " neighbour topologies of topology\n\n" << from->objectRelations
				<< "and selected the following " << neighbours.size() << " topologies of those neighbours:\n\n";

			for (unsigned int i = 0; i < neighbours.size(); ++i)
			{
				oss <<  "\t- Topology " << neighbours[i]->identifier << std::endl;
			}
		}

		LogHelper::logMessage(oss.str(), LOG_INFO);
		LogHelper::logLine();
	}

}

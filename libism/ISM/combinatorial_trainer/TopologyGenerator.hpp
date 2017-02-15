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

#include "ConnectivityChecker.hpp"
#include "Topology.hpp"

namespace ISM {

using boost::filesystem::path;

class TopologyGenerator
{
	public:
		TopologyGenerator(const std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern, int maxNeighbourCount);

		std::vector<TopologyPtr> generateNeighbours(const std::string &pattern, TopologyPtr from);
		std::vector<TopologyPtr> generateStarTopologies(const std::string &pattern);

		TopologyPtr generateFullyMeshedTopology(const std::string &pattern);
		TopologyPtr generateRandomTopology(const std::string &pattern);

		virtual std::string getDescription() = 0;

	private:
		ConnectivityCheckerPtr mConnectivityChecker;

		std::map<std::string, ISM::ObjectRelations> mAllObjectRelationsPerPattern;

		std::vector<std::vector<bool>> selectNeighbours(std::vector<std::vector<bool>>& neighbours);
		TopologyPtr convertBitvectorToTopology(const std::vector<bool> & bitvector, const std::string& pattern);
		std::vector<bool> convertTopologyToBitvector(TopologyPtr topology, const std::string& pattern);

		std::vector<std::vector<bool>> filterBitvectors(std::vector<std::vector<bool>> bitvectors,
				const std::string& pattern);
		std::vector<TopologyPtr> convertBitvectors(std::vector<std::vector<bool>> bitvectors,
				const std::string& pattern);

		void logNeighbourGeneration(const std::vector<TopologyPtr> & neighbours,
				TopologyPtr from, const unsigned int totalNumber);
		
	protected:
		std::map<std::string, unsigned int> mNumRelationsPerPattern;

		unsigned int mUpperRelationLimit;
		int mMaxNeighbourCount;

		virtual std::vector<std::vector<bool>> calculateNeighbours(const std::string &pattern, std::vector<bool> from) = 0;

}; typedef boost::shared_ptr<TopologyGenerator> TopologyGeneratorPtr;
}

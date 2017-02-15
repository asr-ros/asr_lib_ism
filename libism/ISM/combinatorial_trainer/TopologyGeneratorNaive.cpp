/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "TopologyGeneratorNaive.hpp"
#include "../utility/MathHelper.hpp"

namespace ISM {

	TopologyGeneratorNaive::TopologyGeneratorNaive(const std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern,
				int maxNeighbourCount) 
		: TopologyGenerator(allObjectRelationsPerPattern, maxNeighbourCount)
	{
		for (std::map<std::string, unsigned int>::iterator it = mNumRelationsPerPattern.begin();
				it != mNumRelationsPerPattern.end();
				++it)
		{
			unsigned int numRelations = mNumRelationsPerPattern[it->first];
			std::vector<unsigned int> keys;
			for (unsigned int i = 0; i < numRelations; ++i)
			{
				keys.push_back(i);
			}
			mKeysPerPattern[it->first] = keys;
		}
	}


	std::vector<std::vector<bool>> TopologyGeneratorNaive::calculateNeighbours(const std::string &pattern,
			std::vector<bool> from)
	{
		unsigned int numberRelations = from.size() + 1;
		std::vector<std::vector<bool>> topologies;
		unsigned int numAllRelations = mNumRelationsPerPattern[pattern];
		std::vector<unsigned int> keys = mKeysPerPattern[pattern];

		do
		{
			std::vector<bool> vec(numAllRelations, 0);
			for (unsigned int i = 0; i < numberRelations; ++i) {
				vec[keys[i]] = 1;
			}
			topologies.push_back(vec);
		} while (MathHelper::getNextCombination(keys.begin(), keys.begin() + numberRelations, keys.end()));

		return topologies;
	}

	std::string TopologyGeneratorNaive::getDescription()
	{
		std::stringstream s;
		s << "TopologyGenerator is TopologyGeneratorNaive.cpp: " << std::endl
			<< "- Generates all possible relation topologies" << std::endl;
		return s.str();
	}
}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ConnectivityChecker.hpp"
#include <iostream>

namespace ISM
{

	ConnectivityChecker::ConnectivityChecker(std::map<std::string, unsigned int> numObjectsPerPattern)
	{
		mNumObjectsPerPattern = numObjectsPerPattern;
		for (std::map<std::string, unsigned int>::iterator it = numObjectsPerPattern.begin(); it != numObjectsPerPattern.end(); ++it)
		{
			unsigned int numObjects = it->second;

			std::vector<Relation> relations;

			for (unsigned int i = 0; i < numObjects; ++i)
			{
				for (unsigned int j = i + 1; j < numObjects; ++j)
				{
					relations.push_back({i, j});
				}
			}

			mRelationsPerPattern[it->first] = relations;
		}
	}

	bool ConnectivityChecker::isConnected(std::vector<bool> bitvector, const std::string& patternName)
	{
		unsigned int numOnesInVector = 0;
		for (bool bit : bitvector)
		{
			numOnesInVector += bit;
		}

		if (numOnesInVector < mNumObjectsPerPattern[patternName] - 1)
		{
			//A topology needs at least #objects - 1 relations to be connected.
			return false;
		}

		std::stack<unsigned int>().swap(mToVisit);
		mVisited.clear();

		visitNeighbours(patternName, 0, bitvector);

		while(!mToVisit.empty())
		{
			unsigned int toVisit = mToVisit.top();
			mToVisit.pop();
			visitNeighbours(patternName, toVisit, bitvector);
		}

		return mVisited.size() == mNumObjectsPerPattern[patternName];
	}

	void ConnectivityChecker::checkNeighbour(const::std::string &patternName, unsigned int from, unsigned int index, std::vector<bool> bitvector)
	{
		if (bitvector[index])
		{
			int neighbour = mRelationsPerPattern[patternName][index].getNeighbour(from);
			if (mVisited.find(neighbour) == mVisited.end())
			{
				mToVisit.push(neighbour);
			}
		}
	}

	void ConnectivityChecker::visitNeighbours(const std::string& patternName, unsigned int from, std::vector<bool> bitvector)
	{
		mVisited.insert(from);
		unsigned int numObjects = mNumObjectsPerPattern[patternName];
		int index = 0;
		for (unsigned int i = 0; i < from; ++i)
		{
			checkNeighbour(patternName, from, index + from - (i + 1), bitvector);
			index += numObjects - (i + 1);
		}

		for (unsigned int i = index; i < index + numObjects - (from + 1); ++i)
		{
			checkNeighbour(patternName, from, i, bitvector);
		}
	}
}

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
#include <map>
#include <set>
#include <stack>

namespace ISM {

class ConnectivityChecker
{
	public:
		ConnectivityChecker(std::map<std::string, unsigned int> numObjectsPerPattern);
		bool isConnected(std::vector<bool> bitvector, const std::string& patternName);

	private:
		struct Relation
		{
			unsigned int x, y;

			unsigned int getNeighbour(unsigned int z)
			{
				if (z == x)
				{
					return y;
				} else {
					return x;
				}
			}
		};

		std::map<std::string, unsigned int> mNumObjectsPerPattern;
		std::map<std::string, std::vector<Relation>> mRelationsPerPattern;

		std::set<unsigned int> mVisited;
		std::stack<unsigned int> mToVisit;

		void checkNeighbour(const std::string& patternName, unsigned int from, unsigned int index, std::vector<bool> bitvector);
		void visitNeighbours(const std::string& patternName, unsigned int from, std::vector<bool> bitvector);

}; typedef boost::shared_ptr<ConnectivityChecker> ConnectivityCheckerPtr;

}



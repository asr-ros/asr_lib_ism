/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "TopologyGeneratorPaper.hpp"
#include "utility/MathHelper.hpp"

namespace ISM
{

	std::vector<std::vector<bool>> TopologyGeneratorPaper::calculateNeighbours(const std::string &pattern,
			std::vector<bool> from) {

		(void) pattern;

		std::vector<std::vector<bool>> neighbours;
		std::vector<unsigned int> ones;
		std::vector<unsigned int> zeros;

		for (unsigned int i = 0; i < from.size(); ++i)
		{
			if (from[i])
			{
				ones.push_back(i);
			} else {
				zeros.push_back(i);
			}
		}

		std::vector<bool> neighbour;
		if (mRemoveRelations)
		{
			//Remove one relation
			for (unsigned int i = 0; i < ones.size(); ++i)
			{
				neighbour = from;
				neighbour[ones[i]] = 0;
				neighbours.push_back(neighbour);
			}
		}

		for (unsigned int i = 0; i < zeros.size(); ++i)
		{
			//Add one relation
			neighbour = from;
			neighbour[zeros[i]] = 1;
			neighbours.push_back(neighbour);

			if (mSwapRelations) {
				//Swap one relation
				for (unsigned int j = 0; j < ones.size(); ++j)
				{
					neighbour = from;
					neighbour[zeros[i]] = 1;
					neighbour[ones[j]] = 0;
					neighbours.push_back(neighbour);
				}
			}
		}

		return neighbours;

	}

	std::string TopologyGeneratorPaper::getDescription() {
		std::stringstream s;
		s << "TopologyGenerator is TopologyGeneratorPaper.cpp: " << std::endl
			<< "- Generates relation topologies as described in the paper \"Automated Selection of Spatial "
			<<  "Object Relations for Modeling and Recognizing Indoor Scenes with Hierarchical Implicit Shape Models \""
			<< std::endl;
		return s.str();
	}

}

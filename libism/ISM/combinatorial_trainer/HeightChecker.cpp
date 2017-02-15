/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "HeightChecker.hpp"
#include "sstream"
#include "utility/LogHelper.hpp"

namespace ISM {

	bool HeightChecker::isTreeValid(TreePtr tree)
	{
		if (tree->getHeight() > mMaxHeight)
		{
			LogHelper::logMessage("Rejected tree because its height (" + std::to_string(tree->getHeight())
					+ ") is greater than the max height (" + std::to_string(mMaxHeight) + ")\n",
					LOG_INFO, LOG_COLOR_TREE_VALDIATOR_COLOR);
			return false;
		}
		LogHelper::logMessage("Tree is valid \n", LOG_INFO, LOG_COLOR_TREE_VALDIATOR_COLOR);
		return true;
	}

	std::string HeightChecker::getDescription()
	{
		std::stringstream s;
		s << "TreeValidator is HeightChecker.cpp : " << std::endl
			<< "- Checking if tree height is <= " << mMaxHeight
			<< " and if tree is connected" << std::endl;
		return s.str();
	}
}


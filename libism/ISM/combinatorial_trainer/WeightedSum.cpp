/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "WeightedSum.hpp"

namespace ISM {

	double WeightedSum::calculateCost(TopologyPtr instance)
	{
		double normalisedFalsePositives = 
			getNormalisedFalsePositives(instance->evaluationResult.falsePositives);
		double normalisedAverageRecognitionRuntime = 
			getNormalisedAverageRecognitionRuntime(instance->evaluationResult.averageRecognitionRuntime);

		if (normalisedFalsePositives < 0 || normalisedAverageRecognitionRuntime < 0)
		{
			return std::numeric_limits<double>::infinity();
		}

		double cost = mAlpha * normalisedFalsePositives + mBeta * normalisedAverageRecognitionRuntime; 
		instance->cost = cost;
		return cost;
	}

	double WeightedSum::getNormalisedFalsePositives(double falsePositives)
	{
		if (falsePositives > mMaxFalsePositives)
		{
			//False Postivies are higher than the false positves of the worst star topology.
			return -1;
		}

		if (mMaxFalsePositives == mMinFalsePositives)
		{
			return 0;
		}

		return (falsePositives - mMinFalsePositives) / (mMaxFalsePositives - mMinFalsePositives);
	}


	double WeightedSum::getNormalisedAverageRecognitionRuntime(double averageRecognitionRuntime)
	{
		if (averageRecognitionRuntime > mMaxAverageRecognitionRuntime)
		{
			//Average recognition runtime is longer than the average recognition runtime of the fully meshed topology.
			return -1;
		}

		if (mMaxAverageRecognitionRuntime == averageRecognitionRuntime)
		{
			return 0;
		}

		return (averageRecognitionRuntime - mMinAverageRecognitionRuntime) /
			(mMaxAverageRecognitionRuntime - mMinAverageRecognitionRuntime);
	}



}


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
#include <random>
#include "OptimizationAlgorithm.hpp"
#include "NeighbourhoodFunction.hpp"

namespace ISM {

template <class InstanceType>
class HillClimbingAlogrithm : public OptimizationAlgorithm<InstanceType>
{
	public:
		HillClimbingAlogrithm(NeighbourhoodFunctionPtr<InstanceType> neighbourhoodFunction,
			CostFunctionPtr<InstanceType> costFunction, bool randomWalkProbability = 0.0)
			: OptimizationAlgorithm<InstanceType>(neighbourhoodFunction, costFunction)
			, mRandomWalkProbability(randomWalkProbability)
		{}

		InstanceType optimize(InstanceType startInstance)
		{
			InstanceType selectedInstance = startInstance;

			double lowestCost = this->mCostFunction->calculateCost(selectedInstance);
			bool keepOptimizing = true;

			while (keepOptimizing)
			{
				keepOptimizing = false;

				this->mNeighbourhoodFunction->setReferenceInstance(selectedInstance);

				if (this->mNeighbourhoodFunction->hasNextNeighbour())
				{
					//Check if random walk shuld be performed
					if (mDistribution(mGenerator) < mRandomWalkProbability)
					{
							selectedInstance = this->mNeighbourhoodFunction->getNextNeighbour();
							lowestCost = this->mCostFunction->calculateCost(selectedInstance);
							keepOptimizing = true;
					}
					else
					{
						do
						{
							InstanceType nextInstance = this->mNeighbourhoodFunction->getNextNeighbour();
							double nextInstanceCost = this->mCostFunction->calculateCost(nextInstance);

							if (nextInstanceCost < lowestCost)
							{
								selectedInstance = nextInstance;
								lowestCost = nextInstanceCost;
								keepOptimizing = true;
							}
						} while (this->mNeighbourhoodFunction->hasNextNeighbour());
					}
				}
				else
				{
					//The selected instance has no neighbours, we can't continue the optimization
					return selectedInstance;
				}

			}

			return selectedInstance;
		}

	  private:
		bool mRandomWalkProbability;
		std::default_random_engine mGenerator;
		std::uniform_real_distribution<double> mDistribution = std::uniform_real_distribution<double>(0.0, 1.0);
};

template<class InstanceType>
using HillClimbingAlogrithmPtr = boost::shared_ptr<HillClimbingAlogrithm<InstanceType>>;

}

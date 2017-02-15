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
#include "OptimizationAlgorithm.hpp"
#include "NeighbourhoodFunction.hpp"
#include "AcceptanceFunction.hpp"

namespace ISM {

using boost::filesystem::path;

template <class InstanceType>
class RecordHuntAlgorithm : public OptimizationAlgorithm<InstanceType>
{
	public:
		RecordHuntAlgorithm(NeighbourhoodFunctionPtr<InstanceType> neighbourhoodFunction,
			CostFunctionPtr<InstanceType> costFunction,
			AcceptanceFunctionPtr acceptanceFunction)
			: OptimizationAlgorithm<InstanceType>(neighbourhoodFunction, costFunction)
			, mAcceptanceFunction(acceptanceFunction)
		{}

		InstanceType optimize(InstanceType startInstance)
		{

			mAcceptanceFunction->reset();

			InstanceType selectedInstance = startInstance;
			InstanceType bestInstance = startInstance;

			double lowestCost = this->mCostFunction->calculateCost(bestInstance);
			bool keepOptimizing = true;

			while (keepOptimizing)
			{
				keepOptimizing = false;

				this->mNeighbourhoodFunction->setReferenceInstance(selectedInstance);
				double selectedInstanceCost = this->mCostFunction->calculateCost(selectedInstance);


				if (this->mNeighbourhoodFunction->hasNextNeighbour())
				{
					do
					{
						InstanceType nextInstance = this->mNeighbourhoodFunction->getNextNeighbour();
						double nextInstanceCost = this->mCostFunction->calculateCost(nextInstance);

						if (mAcceptanceFunction->isNewCostAcceptable(nextInstanceCost, selectedInstanceCost))
						{
							keepOptimizing = true;
							selectedInstance = nextInstance;
							mAcceptanceFunction->update();

							if (nextInstanceCost < lowestCost)
							{
								bestInstance = nextInstance;
								lowestCost = nextInstanceCost;
							}

							break;
						}

					} while (this->mNeighbourhoodFunction->hasNextNeighbour());
				}
				else
				{
					//The selected instance has no neighbours, we can't continue the optimization
					return bestInstance;
				}
			}

			return bestInstance;
		}

	private:
		AcceptanceFunctionPtr mAcceptanceFunction;

};

template<class InstanceType>
using RecordHuntAlgorithmPtr = boost::shared_ptr<RecordHuntAlgorithm<InstanceType>>;

}

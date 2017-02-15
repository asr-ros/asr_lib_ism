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

#include "ObjectRelation.hpp"
#include "common_type/ObjectSet.hpp"
#include "common_type/RecognitionResult.hpp"
#include <functional>

namespace ISM
{
	struct FalsePositive
	{
		std::pair<unsigned, RecognitionResultPtr> falsePositive;
		std::pair<unsigned, RecognitionResultPtr> correctResult;
		FalsePositive(std::pair<unsigned, RecognitionResultPtr> falsePositive,
				std::pair<unsigned, RecognitionResultPtr>correctResult) : falsePositive(falsePositive),
		correctResult(correctResult) {}
	};

	struct FalseNegative
	{
		std::pair<unsigned, RecognitionResultPtr> falseNegative;
		std::pair<unsigned, RecognitionResultPtr> correctResult;
		FalseNegative(std::pair<unsigned, RecognitionResultPtr>falseNegative,
				std::pair<unsigned, RecognitionResultPtr>correctResult) : falseNegative(falseNegative),
		correctResult(correctResult) {};
	};

	typedef boost::shared_ptr<FalsePositive> FalsePositivePtr;
	typedef boost::shared_ptr<FalseNegative> FalseNegativePtr;

	struct TopologyValidationResult
	{
		std::map<ObjectSetPtr, FalsePositivePtr> falsePositives;
		std::map<ObjectSetPtr, FalseNegativePtr> falseNegatives;
	};
	typedef boost::shared_ptr<TopologyValidationResult> TopologyValidationResultPtr;
	typedef std::map<int, TopologyValidationResultPtr, std::less<unsigned> > TopologyValidationResults;
	typedef boost::shared_ptr<TopologyValidationResults> TopologyValidationResultsPtr;
}

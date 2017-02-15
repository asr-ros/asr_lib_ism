/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ObjectSetValidator.hpp"
#include "utility/LogHelper.hpp"
#include "boost/functional/hash.hpp"

#include <ctime>
#include <sys/time.h>
#include <chrono>

namespace  ISM {

	std::pair<bool, double> ObjectSetValidator::isSetValid(const ObjectSetPtr &testSet,
			const std::string &patternName)
	{
		struct timeval start;
		struct timeval end;

		gettimeofday(&start, NULL);
		std::vector<RecognitionResultPtr> results = mRecognizer->recognizePattern(testSet);
		gettimeofday(&end, NULL);

		double recognitionRuntime, seconds, useconds;
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		recognitionRuntime = seconds + useconds / 1000000;

		for (size_t i = 0; i < results.size(); ++i)
		{
			if (results[i]->patternName == patternName)
			{
				bool isValid = results[i]->confidence >= mConfidenceThreshold;
				return std::make_pair(isValid, recognitionRuntime);
			}
		}

		return std::make_pair(false, recognitionRuntime);
	}

	void ObjectSetValidator::setISM(const IsmPtr ism)
	{
		this->mRecognizer->clearData();
		this->mRecognizer->setVoteSpecifiersPerObject(ism->voteSpecifiersPerObject);
		this->mRecognizer->setObjectTypes(ism->objectTypes);
		this->mRecognizer->setPatternDefinitions(ism->patternDefinitions);
		this->mRecognizer->arrangePatternsAccordingToTreeHeight();
	}
}

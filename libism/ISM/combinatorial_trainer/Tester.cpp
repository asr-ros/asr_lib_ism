/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Tester.hpp"

#include <boost/thread.hpp>

#include "../typedef.hpp"
#include "../utility/LogHelper.hpp"

namespace ISM
{
	EvaluationResult Tester::evaluate(std::string patternName, const IsmPtr& ism)
	{
		LogHelper::logMessage("Evaluating the ISM: ", LOG_INFO, LogHelper::LOG_COLOR_BLUE);

		mObjectSetValidator->setISM(ism);

		mEvaluationRuntime = 0;

		std::vector<ISM::ObjectSetPtr> invalidTestSets = mInvalidTestSetsPerPattern.at(patternName);
		std::vector<ISM::ObjectSetPtr> validTestSets = mValidTestSetsPerPattern.at(patternName);

		mNumTestsDone = 0;
		mNumTests = invalidTestSets.size();

		if (mTestForFalseNegatives)
		{
			mNumTests += validTestSets.size();
		}

		LogHelper::displayProgress(0);
		unsigned falsePositives = evaluateTestSets(invalidTestSets, patternName, false);

		unsigned falseNegatives = 0;
		if (mTestForFalseNegatives)
		{
			falseNegatives = evaluateTestSets(validTestSets, patternName, true);
		}
		LogHelper::displayProgress(1);

		double averageEvaluationDuration = mEvaluationRuntime / mNumTests;

		EvaluationResult er = {falsePositives,
			falseNegatives,
			averageEvaluationDuration};

		LogHelper::logMessage("\n\nEvaluation result is: ", LOG_INFO, LogHelper::LOG_COLOR_BLUE);
		LogHelper::logMessage(er.getDescription(), LOG_INFO, LogHelper::LOG_COLOR_BLUE);

		return er;
	}

	unsigned Tester::evaluateTestSets(const std::vector<ISM::ObjectSetPtr>& testSets, const std::string& patternName, 
			bool expectedValue)
	{
		unsigned numUnexpectedResults = 0;

		for (unsigned i = 0; i < testSets.size(); ++i)	
		{
			std::pair<bool, double> result = mObjectSetValidator->isSetValid(testSets[i], patternName);

			if (expectedValue != result.first)
			{
				numUnexpectedResults++;
			}

			mEvaluationRuntime += result.second;

			mNumTestsDone++;
			LogHelper::displayProgress(((double) mNumTestsDone) / mNumTests);
		}

		return numUnexpectedResults;
	}

	std::string Tester::getDescription() {
		std::stringstream s;
		s << "Evaluator is Tester.cpp:" << std::endl
			<< "- Calcualtes the number of false positives and false negatives and the average evaluation runtime."
			<< std::endl;
		return s.str();
	}
}

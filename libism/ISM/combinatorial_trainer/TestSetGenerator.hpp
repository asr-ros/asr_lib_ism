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

#include <utility>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Geometry>

#include <mutex>

#include "common_type/ObjectSet.hpp"
#include "common_type/Tracks.hpp"

#include "ObjectSetValidator.hpp"
#include "ObjectRelation.hpp"
#include "ImplicitShapeModel.hpp"
#include "utility/LogHelper.hpp"

namespace ISM {

typedef std::map<std::string, std::vector<ISM::ObjectSetPtr> > TestSet;
typedef boost::mt19937                     ENG;    // Mersenne Twister
typedef boost::uniform_int<> DIST;   // Normal Distribution
typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator


class TestSetGenerator
{
	public:
		TestSetGenerator(const std::map<std::string, ISM::ObjectRelations> allObjectRelationsPerPattern,
				ObjectSetValidatorPtr &objectSetValidator)
			: mAllObjectRelationsPerPattern(allObjectRelationsPerPattern)
			, mObjectSetValidator(objectSetValidator)
		{}

		std::pair<std::vector<ObjectSetPtr>, std::vector<ObjectSetPtr>> generateTestSets(const std::string &patternName,
				const TracksPtr& tracks, const IsmPtr ism, unsigned int testSetCount);

	private:
		const std::map<std::string, ISM::ObjectRelations> mAllObjectRelationsPerPattern;
		ISM::ObjectSetValidatorPtr mObjectSetValidator;


		ObjectSetPtr generateRandomObjectSetFromTracks(const ISM::TracksPtr& allTracks,
			const std::string& pattern, GEN &gen);

}; typedef boost::shared_ptr<TestSetGenerator> TestSetGeneratorPtr;

}


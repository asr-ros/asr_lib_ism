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

#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <cstdint>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include "typedef.hpp"
#include "ObjectSet.hpp"
#include "Pose.hpp"
#include "recognizer/VotedPose.hpp"


namespace ISM {

struct RecognitionResult;

typedef boost::shared_ptr<RecognitionResult> RecognitionResultPtr;

/**
     * RecognitionResult class. Instance of a scene. Set of objects that matches scene modeled as ism.
     */
struct RecognitionResult
{
    std::string patternName;
    ///Pose of ism reference in this object configuration.
    PosePtr referencePose;
    ///Objects that have been assigned to scene instance. That match scene model.
    ObjectSetPtr recognizedSet;
    double confidence;
    std::vector<RecognitionResultPtr> subPatterns;
    SummarizedVotedPosePtrs summarizedVotes;

    static constexpr double epsilon = 1e-6;

    RecognitionResult( const std::string patternName, const PosePtr referencePose, const ObjectSetPtr recognizedSet,
                       const double confidence, SummarizedVotedPosePtrs summarizedVotes = SummarizedVotedPosePtrs())
        :
          patternName(patternName),  referencePose(referencePose), recognizedSet(recognizedSet),
          confidence(confidence), summarizedVotes(summarizedVotes), numberOfCombinations(0)
    {}
  
    /**
     * Returns number of combinations directly if already computed
     *                                         else accumulate it through the tree.
     *
     * @return numberOfCombinations represents the number of possible scene configurations which would generate this recognition result.
     */
    u_int64_t getNumberOfCombinations();

private:
    u_int64_t numberOfCombinations;

};

typedef boost::shared_ptr<RecognitionResult> RecognitionResultPtr;

bool operator==(const RecognitionResultPtr& recogRes1, const RecognitionResultPtr& recogRes2);
bool operator==(const RecognitionResult& recogRes1, const RecognitionResult& recogRes2);

std::ostream& operator<<(std::ostream &strm, ISM::RecognitionResult &o);
std::ostream& operator<<(std::ostream &strm, ISM::RecognitionResultPtr &o);
}

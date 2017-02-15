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

#include "BaseRater.hpp"

namespace ISM {
/**
 *  SimpleRater class. The SimpleRater just conisder if certain conditions are met and only rate with 0 or 1.
 */
class SimpleRater : public BaseRater
{
public:
    SimpleRater(double bin_size, double maxAngleDeviation) : BaseRater(bin_size, maxAngleDeviation){}

    //Methods are inlined for performance reasons.

    // No input checks at all since that method must run fast.
    virtual double rateAtBackProjectionLevel(VotedPosePtr& votedPose, RatingDataPtr data)
    {
      //Ugly but fast. And we need to be as fast as possible.
      data == nullptr;

      return votedPose->weight;
    }

  virtual void printRatingAtBackProjectionLevel(VotedPosePtr& votedPose, PosePtr& refPose, RatingDataPtr data)
    {

      if (data == nullptr)
	data = RatingDataPtr(new RatingData());

      double score;

      if(this->isVoteSupportingReference(votedPose, refPose, data))
	score = votedPose->weight;
      else
	score = 0.0;
 
      std::cout << "SimpleRater score=" << score
		<< "\nNormalized score=" << score/votedPose->weightDenominator << " with denominator: " << votedPose->weightDenominator << std::endl;

      return;
    }

};
}

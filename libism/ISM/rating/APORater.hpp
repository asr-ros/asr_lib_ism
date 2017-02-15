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
 *  APORater class. The APORater consider appearance, position and orientation for the rating.
 */
class APORater : public BaseRater
{

public:
    APORater(double bin_size, double maxAngleDeviation) : BaseRater(bin_size, maxAngleDeviation)
    {
      //We calculated a factor for sigma so that exp() is 0.5 at sqrt(3)*/(bin_size*2).
      // https://www.wolframalpha.com/input/?i=e%5E(((-(sqrt(3)%2F2*s)%5E2))%2F(2*(sigma)%5E2))%3D0.5
      squaredSigma = -0.735534 * bin_size * -0.735534 * bin_size;
    }

  //Methods are inlined for performance reasons.

  // No input checks at all since that method must run fast.
  virtual double rateAtBackProjectionLevel(VotedPosePtr& votedPose, RatingDataPtr data){

    /* Note: rate position with density function of a normal distribution without the scaling factor,
       so that the score is in [0,1] */
    double positionScore;
    positionScore = exp(-0.5 * (data->backProjectionSquaredDistance / squaredSigma));

    /* Note: rate orientation with a customized cosinus function, which resemble the density function above
       and the score is in [0,1]*/
    double orientationScore;

    //We doubled the limit for cos() so that this function is 0.5 at angleDeviation == maxAngleDeviation.
    // doubled area of angles with values > 0
    orientationScore = (data->angleDeviation < 2 * maxAngleDeviation) ? 0.5 + 0.5 * cos((M_PI * data->angleDeviation) / ( 2 * maxAngleDeviation)) : 0.0;

    double poseScore = positionScore * orientationScore;

    double totalScore = votedPose->weight - 1.0 + poseScore;

    /*Option to rate appearance in the furture*/
    //double appearanceScore = 1.0;
    //totalScore *= appearanceScore;

    return totalScore;
    
  }

  virtual void printRatingAtBackProjectionLevel(VotedPosePtr& votedPose, PosePtr& refPose, RatingDataPtr data){

    if (data == nullptr)
      data = RatingDataPtr(new RatingData());

    if (!this->isVoteSupportingReference(votedPose, refPose, data))
      {
	std::cout << "Doesn't support reference -> APORater score=0.0" << std::endl;
	return;
      }

    /* Note: rate position with density function of a normal distribution without the scaling factor,
       so that the score is in [0,1] */
    double positionScore;
    positionScore = exp(-0.5 * (data->backProjectionSquaredDistance / squaredSigma));

    /* Note: rate orientation with a customized cosinus function, which resemble the density function above
       and the score is in [0,1]*/
    double orientationScore;

    //We doubled the limit for cos() so that this function is 0.5 at angleDeviation == maxAngleDeviation.
    // doubled area of angles with values > 0
    orientationScore = (data->angleDeviation < 2 * maxAngleDeviation) ? 0.5 + 0.5 * cos((M_PI * data->angleDeviation) / ( 2 * maxAngleDeviation)) : 0.0;

    double poseScore = positionScore * orientationScore;

    double totalScore;
    if (votedPose->source->type.find("_sub") != std::string::npos)  //sub-ism
      {
        totalScore = votedPose->weight - 1.0 + poseScore;
      }
    else //object
      {
        totalScore = poseScore;
        /*Option to rate appearance in the furture*/
        //double appearanceScore = 1.0;
        //totalScore = appearanceScore * poseScore;
      }

    std::cout << "APORater score=" << totalScore << " with"
	      << "\n\tPositionScore=" << positionScore << " with squaredDistance=" << data->backProjectionSquaredDistance << "  maxSquaredDistancepDeviation=" << (squaredSigma * 9) << "  squaredSigma=" << squaredSigma
	      << "\n\tOrientationScore=" << orientationScore << " with angleDeviation=" << data->angleDeviation << "  maxAngleDeviation=" << maxAngleDeviation
	      << "\nNormalized score=" << totalScore/votedPose->weightDenominator << " with denominator: " << votedPose->weightDenominator << std::endl;
 
    return;
    
  }

private:
    double squaredSigma;
};
}

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

#include "typedef.hpp"
#include "common_type/Object.hpp"
#include "recognizer/VotedPose.hpp"
#include "RatingData.hpp"
#include "utility/GeometryHelper.hpp"

namespace ISM {
/**
 *  BaseRater abstract class. Is the base class for rater classes,
 *  which provide methods to rate/score the recognition result throughout the whole recognition process.
 */
class BaseRater
{

  typedef GeometryHelper GH;

public:
        //This is what we "actually" do here: const double maxDistanceInsideBin = sqrt(3.0) * bin_size / 2;
    BaseRater(double bin_size, double maxAngleDeviation) : maxSquaredDistance(0.75 * bin_size * bin_size),
                                    maxAngleDeviation(maxAngleDeviation),
                                    projectedPose(PosePtr(new Pose())),
                                    projectedPoint(PointPtr(new Point()))
    { }

   //Methods are inlined for performance reasons.

   bool isVoteSupportingReference(VotedPosePtr& votedPose, PosePtr& refPose, RatingDataPtr data)
    {
        //Predict position of source object for current vote, using originForFitting.
         GH::getSourcePoint(refPose,
                votedPose->vote->refToObjectQuat,
                votedPose->vote->radius,
                projectedPoint);

        //Calculate distance between positions of real source object and predicted source object.
        double backProjectionSquaredDistance = GH::getSquaredDistanceBetweenPoints(votedPose->source->
                                                                                   pose->point,
                                                                                   projectedPoint);
	//Now also predict the orientation, not only the position.
	GH::getSourcePose(refPose, projectedPoint, votedPose->vote->refToObjectPoseQuat, projectedPose);

        //Calculate angle between orientation of real source object and predicted source object.
        double angleDeviation = GH::getAngleBetweenQuats(votedPose->source->pose->quat, projectedPose->quat);

        //Numerical stuff.
        if (GH::checkIfDoubleNaN(backProjectionSquaredDistance))
            backProjectionSquaredDistance = 0.0;
        if (GH::checkIfDoubleNaN(angleDeviation))
            angleDeviation = 0.0;

        //cache data for further processing
        data->angleDeviation = angleDeviation;
        data->backProjectionSquaredDistance = backProjectionSquaredDistance;

        //Reject all votes whose deviation form originForFitting exceed given thresholds. They cannot be regarded as supporting originForFitting.
        return (backProjectionSquaredDistance <= maxSquaredDistance && angleDeviation < maxAngleDeviation);

    }
  
  //Separated pure calculation of ratings from printing for performance reasons.
  virtual double rateAtBackProjectionLevel(VotedPosePtr& votedPose, RatingDataPtr data) = 0;
  virtual void printRatingAtBackProjectionLevel(VotedPosePtr& votedPose, PosePtr& refPose, RatingDataPtr data) = 0;

protected:
    const double maxSquaredDistance;
    const double maxAngleDeviation;
    PosePtr projectedPose;
    PointPtr projectedPoint;

};

typedef boost::shared_ptr<BaseRater> RaterPtr;
}

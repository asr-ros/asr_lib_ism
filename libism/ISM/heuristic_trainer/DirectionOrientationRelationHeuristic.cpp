/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "DirectionOrientationRelationHeuristic.hpp"

#include <vector>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include "utility/GeometryHelper.hpp"

namespace ISM {
    DirectionOrientationRelationHeuristic::DirectionOrientationRelationHeuristic(const double pStaticBreakRatio, const double pTogetherRatio, const double pMaxAngleDeviation) :
      Heuristic("DirectionOrientationRelationHeuristic"), mStaticBreakRatio(pStaticBreakRatio), mTogetherRatio(pTogetherRatio), mMaxAngleDeviation(pMaxAngleDeviation) {}

    void 
    DirectionOrientationRelationHeuristic::applyHeuristic(const TracksPtr& tracks) {
        typedef GeometryHelper GH;
        typedef Eigen::Vector3d Vector;

        double bestDistance;
        for (TrackPtr& first : tracks->tracks) {
            TrackPtr currentBest;
            double currentClosestDistance;
            int currentBestBreaks = 1;
            int currentBestCommonPositions = 1;

            for (TrackPtr& second : tracks->tracks) {
                if (first == second) {
                    continue;
                }

                /*
                 * What we do:
                 * Calculate a direction Vector from first to second for every Frame.
                 * Check in every frame the angle between the reference vector (first vector) and the current vector.
                 * If the misalignment is more than mMaxAngleDeviation degrees, increase staticBreaks and
                 * recalculate the reference vote.
                 * Also calculate the average distance between first and second.
                 *
                 * At the end, if the staticBreaks are below mStaticBreakRatio of the sample range,
                 * and they appear together in more than mTogetherRatio of the frames,
                 * and the second is closer to first than the current closest track,
                 * replace the current closest track with second.
                 *
                 * At the end, choose the first<->second combination with the lowest rate of static breaks.
                 *
                 * This will always create a cluster of two tracks.
                 */

                int commonPositions = 0;
                double averageDistance = 0;

                for (size_t i = 0; i < first->objects.size(); i++) {
                    ObjectPtr firstObject = first->objects[i];
                    ObjectPtr secondObject = second->objects[i];
                    if (!firstObject || !secondObject) {
                        continue;
                    }

                    averageDistance += GH::getDistanceBetweenPoints(firstObject->pose->point,
                            secondObject->pose->point);
                    commonPositions++;
                }
                if (commonPositions < (double) first->objects.size() * mTogetherRatio) {
                    continue;
                }

                averageDistance /= (double) commonPositions;

                int staticBreaks = 0;
                Vector directionVector;
                double orientationDifference;
                bool firstRun = true;

                for (size_t i = 0; i < first->objects.size(); i++)
                {
                    ObjectPtr firstObject = first->objects[i];
                    ObjectPtr secondObject = second->objects[i];
                    if (!firstObject || !secondObject) {
                        continue;
                    }

                    if (firstRun) {
              directionVector = GH::getDirectionVector(firstObject->pose, secondObject->pose);
                        orientationDifference = GH::getAngleBetweenQuats(
                            firstObject->pose->quat, secondObject->pose->quat
                        );
                        firstRun = false;
                        continue;
                    }

                    Vector currentDirection = GH::getDirectionVector(firstObject->pose, secondObject->pose);
                    double currentOrientationDiff = GH::getAngleBetweenQuats(firstObject->pose->quat,
                                                                           secondObject->pose->quat);

		    double deviation = GH::getAngleBetweenAxes(directionVector, currentDirection);

                    double orientationDeviation = fabs(orientationDifference - currentOrientationDiff);

                    if (deviation > mMaxAngleDeviation || orientationDeviation > mMaxAngleDeviation)
                    {
                        staticBreaks++;
                        directionVector = currentDirection;
                        orientationDifference = currentOrientationDiff;
                    }
                }

                if (
                    ((double) staticBreaks < ((double) commonPositions) * mStaticBreakRatio) &&
                    (!currentBest || (currentClosestDistance > averageDistance))
                    ) {
                    currentBest = second;
                    currentClosestDistance = averageDistance;
                    currentBestBreaks = staticBreaks;
                    currentBestCommonPositions = commonPositions;
                }
            }

            if (currentBest) {
                double conf = 1 - (double) currentBestBreaks / (double) currentBestCommonPositions;
                if (!this->cluster
                    || (conf > this->confidence
                        || (conf == this->confidence && bestDistance > currentClosestDistance))) {
                    std::vector<TrackPtr> cluster;
                    cluster.push_back(first);
                    cluster.push_back(currentBest);
                    this->cluster = TracksPtr(new Tracks(cluster));
                    this->confidence = conf;
                    bestDistance = currentClosestDistance;
                }
            }
        }
    }

}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "StaticRelationHeuristic.hpp"

#include <vector>
#include "utility/GeometryHelper.hpp"

namespace ISM {
    StaticRelationHeuristic::StaticRelationHeuristic(const TracksPtr& tracks) : Heuristic("StaticRelationHeuristic") {
        typedef GeometryHelper GH;

        for (TrackPtr& first : tracks->tracks) {
            TrackPtr currentBest;
            int bestStaticBreaks = 0;
            int bestCommonPositions = 0;
            double bestAvgDistance = 0;

            for (TrackPtr& second : tracks->tracks) {
                if (first == second) {
                    continue;
                }

                /*
                 * What we do:
                 * Calculate a vote from second to first (using first as the reference pose).
                 * Use that vote on the next frame to search first, looking from second.
                 * If the miss alignment is more than 10% of the average position difference between first and second,
                 * incease staticBreaks and recalculate the vote.
                 *
                 * At the end, if the staticBreaks are below 5% of the sample range,
                 * and they appear together in more than 95% of the frames,
                 * add second to the cluster with first as the reference point.
                 */

                VoteSpecifierPtr vote;
                int commonPositions = 0;
                int staticBreaks = 0;
                double averageDistance = 0;

                for (size_t i = 0; i < first->objects.size(); i++) {
                    ObjectPtr firstObject = first->objects[i];
                    ObjectPtr secondObject = second->objects[i];
                    if (!firstObject || !secondObject) {
                        continue;
                    }

                    averageDistance += GH::getDistanceBetweenPoints(
                                    firstObject->pose->point,
                                    secondObject->pose->point
								    );
                    commonPositions++;
                }

                averageDistance /= (double)commonPositions;

                double maxDeviation = averageDistance * 0.1;

                for (size_t i = 0; i < first->objects.size(); i++) {
                    ObjectPtr firstObject = first->objects[i];
                    ObjectPtr secondObject = second->objects[i];
                    if (!firstObject || !secondObject) {
                        continue;
                    }

                    if (!vote) {
                        vote = GH::createVoteSpecifier(secondObject->pose, firstObject->pose);
                        continue;
                    }

                    PointPtr referencePoint = GH::applyQuatAndRadiusToPose(
                        secondObject->pose,
                        vote->objectToRefQuat,
                        vote->radius
                    );

                    double distance = GH::getDistanceBetweenPoints(
                                   firstObject->pose->point,
								   referencePoint
								   );

                    if (distance > maxDeviation) {
                        staticBreaks++;
                        vote = GH::createVoteSpecifier(secondObject->pose, firstObject->pose);
                    }
                }

                if (
                    (double)staticBreaks < ((double)commonPositions) * 0.05 &&
                    commonPositions > (double)first->objects.size() * 0.5 &&
                    (!currentBest || staticBreaks < bestStaticBreaks ||
                            (staticBreaks == bestStaticBreaks && bestAvgDistance > averageDistance))
                ) {
                    currentBest = second;
                    bestStaticBreaks = staticBreaks;
                    bestCommonPositions = commonPositions;
                    bestAvgDistance = averageDistance;
                }
            }

            if (currentBest) {
                double conf = 1 - (double)bestStaticBreaks / (double)bestCommonPositions;
                if (conf > this->confidence) {
                    std::vector<TrackPtr> cluster;
                    cluster.push_back(first);
                    cluster.push_back(currentBest);
                    this->cluster = TracksPtr(new Tracks(cluster));
                    this->confidence = conf;
                }
            }
        }
    }
}

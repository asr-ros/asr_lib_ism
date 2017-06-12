/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "VotingSpace.hpp"

//Global includes
#include <iostream>
#include <stdexcept>
#include <stdlib.h>
#include <math.h>
#include <map>
#include <algorithm>

//Pkg includes
#include <boost/make_shared.hpp>
#include <boost/range/irange.hpp>

//Local includes
#include "utility/GeometryHelper.hpp"
#include "common_type/ObjectSet.hpp"
#include "VotingResult.hpp"
#include "VotingResultCalculator.hpp"

namespace ISM {

VotingResultPtrs VotingSpace::fillAndEvalVotingSpace(VotedPosePtrs& votes, bool enabledSelfVoteCheck)
{
    //Flush voxelgrid from votes of e.g. last sub-ISM that used the grid to process its VotedPoses.
    voteGrid.clear();
    VotingSpacePtr votingSpaceInEval = shared_from_this();
    std::vector<VotingResultCalculatorPtr> calculatorPerThread;
    std::vector<boost::shared_ptr<boost::thread>> threads;
    for (int i : boost::irange(0, nThreads)) {
        votingBinsPerThread[i].clear();
        resultsPerThread[i].clear();
        calculatorPerThread.push_back(VotingResultCalculatorPtr(new VotingResultCalculator(votingSpaceInEval, enabledSelfVoteCheck, mRaterType)));
    }

    std::sort(votes.begin(), votes.end(), [](const VotedPosePtr& v1, const VotedPosePtr& v2)
    {
        return v1->weight > v2->weight;
    });

    curThread = 0;
    //Insert all voted poses into voxelgrid (votingspace).
    for (const VotedPosePtr& vote : votes)
    {
        PointPtr point = vote->pose->point;
        VotingBinPtr bin = getBin(point->eigen.x(), point->eigen.y(), point->eigen.z());
        bin->insert(vote);
    }

    VotingResultPtrs results;
    for (int i : boost::irange(0, nThreads)) {
        //This little guy searches for scene instances in each voxel grid element.
        threads.push_back(boost::make_shared<boost::thread>(&VotingSpace::calcResult, this, i, calculatorPerThread[i]));
    }

    // join all threads and collect results
    for (int i : boost::irange(0, nThreads)) {
        threads[i]->join();
        if (!resultsPerThread[i].empty())
            results.insert(results.end(), resultsPerThread[i].begin(), resultsPerThread[i].end());
    }

    return results;
}

void VotingSpace::calcResult(int i, VotingResultCalculatorPtr& calculator) {
    if (votingBinsPerThread[i].empty())
        return;
    for (VotingBinPtr& votingBin : votingBinsPerThread[i]) {
        VotingResultPtr result;
        if (calculator->computeVotingResult(result, votingBin->votes))
        {
            resultsPerThread[i].push_back(result);
        }
    }
}

VotingBinPtr VotingSpace::getBin(double x, double y, double z)
{
    int binx = discretizeToBins(x);
    int biny = discretizeToBins(y);
    int binz = discretizeToBins(z);

    if (voteGrid.find(binx) == voteGrid.end())
    {
        voteGrid.insert(std::make_pair(binx, YIndexToZIndex()));
    }

    if (voteGrid[binx].find(biny) == voteGrid[binx].end())
    {
        voteGrid[binx].insert(std::make_pair(biny, ZIndexToVotingBinPtr()));
    }

    if (voteGrid[binx][biny].find(binz) == voteGrid[binx][biny].end())
    {
        voteGrid[binx][biny].insert(std::make_pair(binz, VotingBinPtr(new VotingBin(binx, biny, binz))));
        // add bins to thread
        votingBinsPerThread[curThread].push_back(voteGrid[binx][biny][binz]);
        curThread = (curThread + 1) % nThreads;
    }

    return voteGrid[binx][biny][binz];
}

TypeToInnerMap VotingSpace::collectVotesInSphere(PointPtr &sphereCenter, bool& voteFromReferenceObjectExists)
{
    voteFromReferenceObjectExists = false;

    const int binXIdx = discretizeToBins(sphereCenter->eigen.x());
    const int binYIdx = discretizeToBins(sphereCenter->eigen.y());
    const int binZIdx = discretizeToBins(sphereCenter->eigen.z());

    //We need the cartisian position of the center of the current bin to compare it to originForFitting.
    PointPtr binCenter = PointPtr(new Point(binXIdx * binSize, binYIdx * binSize, binZIdx * binSize));

    // 3 bins per axis
    int minXIdx = binXIdx - 1;
    int minYIdx = binYIdx - 1;
    int minZIdx = binZIdx - 1;
    int maxXIdx = binXIdx + 1;
    int maxYIdx = binYIdx + 1;
    int maxZIdx = binZIdx + 1;

    // if sphere center is properly positioned, we don't have to consider all bins
    if (sphereCenter->eigen.x() - binCenter->eigen.x() > offsetOfSphere) {
        minXIdx ++;
    } else if (sphereCenter->eigen.x() - binCenter->eigen.x() < -offsetOfSphere) {
        maxXIdx --;
    }
    if (sphereCenter->eigen.y() - binCenter->eigen.y() > offsetOfSphere) {
        minYIdx ++;
    } else if (sphereCenter->eigen.y() - binCenter->eigen.y() < -offsetOfSphere) {
        maxYIdx --;
    }
    if (sphereCenter->eigen.z() - binCenter->eigen.z() > offsetOfSphere) {
        minZIdx ++;
    } else if (sphereCenter->eigen.z() - binCenter->eigen.z() < -offsetOfSphere) {
        maxZIdx --;
    }

    // NOTE: At most 3 * 3 * 3 = 27 bins must be checked
    TypeToInnerMap currentVoteMap;

    //Parts of this code are executed millions of times, so this code is perfomance optimized, not beautiful.
    for (int xIdx = minXIdx; xIdx <= maxXIdx; xIdx++)
    {
        XIndexToYIndex::iterator yzGridIt = voteGrid.find(xIdx);
        if (yzGridIt == voteGrid.end()) {
            continue;
        }
        YIndexToZIndex yzGrid = yzGridIt->second;

        for (int yIdx = minYIdx; yIdx <= maxYIdx; yIdx++)
        {
            YIndexToZIndex::iterator zGridIt = yzGrid.find(yIdx);
            if (zGridIt == yzGrid.end()) {
                continue;
            }
            ZIndexToVotingBinPtr zGrid = zGridIt->second;

            for (int zIdx = minZIdx; zIdx <= maxZIdx; zIdx++)
            {
                ZIndexToVotingBinPtr::iterator votingBinIt = zGrid.find(zIdx);
                if (votingBinIt == zGrid.end()) {
                    continue;
                }
                VotingBinPtr votingBinPtr =  votingBinIt->second;

		VotedPosePtr votedPosePtr;
		double squaredDistance;

		//Check all type and id combinations in that voxel element.
                for (TypeToInnerMap::iterator typeIt = votingBinPtr->votes.begin();
                     typeIt != votingBinPtr->votes.end(); typeIt++)
                {

		  for (IdToVoteMap::iterator idIt = typeIt->second.begin();
		       idIt != typeIt->second.end(); idIt++)
                    {
		      //Categorizing votedPoses per type and id is required for reducing optimization of votedPose combination to optimization of votedPoses per type and id combination.
              VotedPosePtrs& mapForCurrentTypeAndId = currentVoteMap[typeIt->first][idIt->first];

              //Self vote needs only to be checked once per type and id combination since all votes of the reference object are selfvotes.
              if (GeometryHelper::isSelfVote(idIt->second.front()->vote))
                voteFromReferenceObjectExists = true;

		      for (VotedPosePtrs::iterator votedPosePtrsIt = idIt->second.begin();
			   votedPosePtrsIt != idIt->second.end(); votedPosePtrsIt++)
                        {
			  votedPosePtr = (*votedPosePtrsIt);
			  squaredDistance = GeometryHelper::getSquaredDistanceBetweenPoints(sphereCenter,
											votedPosePtr->pose->point);
			  //Does votedPose lie within sphere?
			  if (squaredDistance < squaredRadius)
			    mapForCurrentTypeAndId.push_back(votedPosePtr);
                        }
                    }
                }
            }
        }
    }
    return currentVoteMap;
}

int VotingSpace::discretizeToBins(double x)
{
    int result = 0;

    if(x >= -halfBinSize)
    {
        double temp = ((x + halfBinSize) / binSize);
        if(fabs(temp - round(temp)) < VotingSpace::epsilon)
	  result = (int)round(temp);
        else
	  result = (int) temp;
    }
    else
    {
        // 0.7071067811865475 double should have precision 15
        double temp = (x - halfBinSize) / binSize;
        if(fabs(temp - round(temp)) < VotingSpace::epsilon)
            result = ((int)round(temp)) + 1;
        else
            result = (int) temp;
    }
    return result;
}

}

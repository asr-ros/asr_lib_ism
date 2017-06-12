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
//Pkg includes
//NOTE: Has to be included to enable the class to give a shared pointer of itself the an other function.
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/thread.hpp>

//Local includes
#include "typedef.hpp"
#include "common_type/Point.hpp"
#include "VotedPose.hpp"
#include "common_type/Object.hpp"
#include "VotingBin.hpp"

namespace ISM {

//Computed VotedPoses (reference pose votes) are inserted into voxelgrid and each voxelgrid element is searched for promising combinations of votes.
class VotingSpace : public boost::enable_shared_from_this<VotingSpace>
{

public:

  VotingSpace(double binSize, double maxProjectionAngleDeviation, int raterType = 0) :
        enable_shared_from_this(),
        binSize(binSize),
        maxProjectionAngleDeviation(maxProjectionAngleDeviation),
	mRaterType(raterType),
	halfBinSize(binSize/(2.0)),
    squaredRadius((3.0 / 4.0) * binSize * binSize),
        offsetOfSphere((sqrt(3) * binSize / 2.0) - halfBinSize),
    nThreads(boost::thread::hardware_concurrency()),
        threads(nThreads),
        votingBinsPerThread(nThreads),
        resultsPerThread(nThreads)
    { }

    //Insertion into grid and its evaluation. This method is called for each sub-ISM at a time.
    VotingResultPtrs fillAndEvalVotingSpace(VotedPosePtrs& votes, bool enabledSelfVoteCheck);
    //Converts from cartesian to grid coordinates.
    VotingBinPtr getBin(double x, double y, double z);
    //Implements radius search during votingspace evaluation.
    TypeToInnerMap collectVotesInSphere(PointPtr& centerPtr, bool& voteFromReferenceObjectExists);

    int discretizeToBins(double x);

    void calcResult(int i, VotingResultCalculatorPtr& calculator);

    //This is the actual grid, wrapped in this class.
    XIndexToYIndex voteGrid;

    //Currently accepted deviations for scene recognition with the given grid.
    double binSize;
    double maxProjectionAngleDeviation;

    //Simple or APO-Rater enum.
    const int mRaterType;

    //Some members for discretizeToBins.
    const double halfBinSize;
    static constexpr double epsilon = 1e-6;

    //Size of sphere for radius search.
    // NOTE: const double radius = sqrt(3.0) * binSize / 2; Perform  with diameter = bin diagonal around centerPtr = originForFitting.
    const double squaredRadius;
    //The sphere is larger than bin size.
    const double offsetOfSphere;

    // per thread data/maybe extract this to a class
    int nThreads;
    std::vector<boost::shared_ptr<boost::thread>> threads;
    std::vector<std::vector<VotingBinPtr>> votingBinsPerThread;
    std::vector<std::vector<VotingResultPtr>> resultsPerThread;
    int curThread;
};
}

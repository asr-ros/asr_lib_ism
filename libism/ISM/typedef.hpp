/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TYPEDEF_HPP
#define TYPEDEF_HPP

#include <set>
#include <vector>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
/*!
* \brief this namespace contains all generally usable classes.
*/
namespace ISM {
  // Forward declaration
  struct VotedPose;
  typedef boost::shared_ptr<VotedPose> VotedPosePtr;

  class VotingSpace;
  typedef boost::shared_ptr<VotingSpace> VotingSpacePtr;

  struct VotingResult;
  typedef boost::shared_ptr<VotingResult> VotingResultPtr;

  struct Point;
  typedef boost::shared_ptr<Point> PointPtr;

  struct VotingBin;
  typedef boost::shared_ptr<VotingBin> VotingBinPtr;

  struct Tree;
  typedef boost::shared_ptr<Tree> TreePtr;

  struct LogHelper;
  typedef boost::shared_ptr<LogHelper> LogHelperPtr;

  struct ObjectSet;
  typedef boost::shared_ptr<ObjectSet> ObjectSetPtr;

  class VotingResultCalculator;
  typedef boost::shared_ptr<VotingResultCalculator> VotingResultCalculatorPtr;

  //Global typedefs
  typedef std::vector<VotedPosePtr> VotedPosePtrs;
  typedef std::vector<VotingResultPtr> VotingResultPtrs;
  typedef std::vector<PointPtr> PointPtrs;

  typedef std::pair<std::string, VotingSpacePtr> PatternNameAndVotingSpaceTuple;

  typedef std::map<std::string, VotedPosePtrs> PatternNameToVotedPoses;
  typedef std::map<unsigned int, std::set<std::string> > TreeHeightToPatternName;

  typedef std::map<std::string, std::vector<ObjectSetPtr> > PatternNameToObjectSet;

  typedef std::map<std::string, VotedPosePtrs> IdToVoteMap;
  typedef std::map<std::string, IdToVoteMap> TypeToInnerMap;

  typedef std::pair<unsigned int, double> CountWeightPair;
  typedef std::pair<VotedPosePtr, CountWeightPair> SummarizedVotedPosePtr;
  typedef std::vector<SummarizedVotedPosePtr> SummarizedVotedPosePtrs;

  typedef std::map<int, VotingBinPtr> ZIndexToVotingBinPtr;
  typedef std::map<int, ZIndexToVotingBinPtr> YIndexToZIndex;
  typedef std::map<int, YIndexToZIndex> XIndexToYIndex;
}

#endif // TYPEDEF_HPP

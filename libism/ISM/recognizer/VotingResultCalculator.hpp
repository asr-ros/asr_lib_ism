/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <map>
#include <stack>
#include <set>
#include <vector>
#include <iostream>

//Local includes
#include "common_type/Pattern.hpp"
#include "common_type/ObjectSet.hpp"
#include "VotingSpace.hpp"
#include "VotingResult.hpp"
#include "rating/SimpleRater.hpp"
#include "rating/APORater.hpp"
#include "rating/RatingData.hpp"
#include "typedef.hpp"

namespace ISM {
  //Searches scene instances in given sets of votes and lets VotingSpace perform radius search if that features is requested.
  class VotingResultCalculator
  {
  public:

    VotingResultCalculator(VotingSpacePtr votingSpaceInEval, int raterType = 0);

    //Compute VotingResults (result) from a set of VotedPoses (votedReferencePoses). Calls searchFittingVotes.
    bool computeVotingResult(VotingResultPtr& result, TypeToInnerMap votedReferencePoses);

  private:

    //Matches given votes to a originForFitting, chosen beforehand.
    void searchFittingVotes(TypeToInnerMap votes);

    //Voting space used for radius search.
    VotingSpacePtr mVotingSpaceInEval;
    //Simple or APO-Rater
    ISM::RaterPtr mRater;

    //VotedPose for reference, used as starting point (reference) for fitting process (either in bin or in sphere).
    VotedPosePtr originForFitting;
    //The pose inside originForFitting.
    PosePtr originPose;
    //Used to recognize votes from originForFitting
    std::string originType;
    std::string originId;

    //All votedPoses that fit current originForFitting are counted and added to RecognitionResult.
    SummarizedVotedPosePtrs summarizedFittingVotes;

    //Cache data for rating to prevent calculating already calculated data again.
    RatingDataPtr ratingCache;

    //To prevent that the same object votes multiple types when evaluating an originForFitting (could happen in ism tree during iterative scene recognition).
    std::set<ObjectPtr> sourcesWithFittingVotes;

    //Both following methods inlined for performance reasons.

    //For the type and id combination providing originForFitting: Calculate all its votes that fit.
    unsigned int countSupportingOriginForFittingVotes(VotedPosePtrs originForFittingVotes)
    {
      
      VotedPosePtr currentVotedPose;
      unsigned int fittingVoteCounter = 0;

      //Go through all votes for current type and id combination.
      for (VotedPosePtrs::iterator voteIt = originForFittingVotes.begin(); voteIt != originForFittingVotes.end(); voteIt++)
	{
	  currentVotedPose = *voteIt;
	  //Prevent that fitting takes place multiple times for the same object when using the same originForFitting.
	  if ((sourcesWithFittingVotes.find(currentVotedPose->source) != sourcesWithFittingVotes.end()))
	    continue;

	  //Reject all votes whose deviation form originForFitting exceed given thresholds. They cannot be regarded as supporting originForFitting.
	  if (mRater->isVoteSupportingReference(currentVotedPose, originPose, ratingCache))
	    //Count it as a supporting vote for origin for fitting
	    fittingVoteCounter++;
	}

      return fittingVoteCounter;

    }

    //For a given type and id combination: Calculate all parameters required for summarizedFittingVotes
    void countSupportingOriginForFittingVotes(VotedPosePtrs votes, unsigned int& fittingVoteCounter, double& bestWeight, VotedPosePtr& bestVotedPose)
    {
    
      VotedPosePtr currentVotedPose;  
      double newWeight;
      fittingVoteCounter = 0;
      bestWeight = -mVotingSpaceInEval->epsilon;
  
      //Go through all votes for current type and id combination.
      for (VotedPosePtrs::iterator voteIt = votes.begin(); voteIt != votes.end(); voteIt++)
	{
	  currentVotedPose = *voteIt;
	  //Prevent that fitting takes place multiple times for the same object when using the same originForFitting.
	  if ((sourcesWithFittingVotes.find(currentVotedPose->source) != sourcesWithFittingVotes.end()))
	    continue;

	  //Reject all votes whose deviation form originForFitting exceed given thresholds. They cannot be regarded as supporting originForFitting.
	  if (mRater->isVoteSupportingReference(currentVotedPose, originPose, ratingCache))
	    {
	      //Count it as a supporting vote for origin for fitting
	      fittingVoteCounter++;
	      //If we already found a perfectly rated VotedPose for the current type and id, we do not need to rate the rest.
	      if(bestWeight >= currentVotedPose->weight - VotingSpace::epsilon)
		continue;

	      newWeight = mRater->rateAtBackProjectionLevel(currentVotedPose, ratingCache);
	      if(newWeight > bestWeight)
		{
		  bestVotedPose = currentVotedPose;
		  bestWeight = newWeight;
		}
	    }
	}
    }

  };

  typedef boost::shared_ptr<VotingResultCalculator> VotingResultCalculatorPtr;

}



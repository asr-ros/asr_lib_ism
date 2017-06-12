/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "VotingResultCalculator.hpp"
#include "utility/GeometryHelper.hpp"

namespace ISM {

  typedef GeometryHelper GH;

  VotingResultCalculator::VotingResultCalculator(VotingSpacePtr votingSpaceInEval, bool enableSelfVoteCheck, int raterType) : mVotingSpaceInEval(votingSpaceInEval), enabledSelfVoteCheck(enableSelfVoteCheck)
  
  {

      ratingCache = RatingDataPtr(new RatingData());

      switch (raterType) {
      case 1:
	this->mRater = ISM::RaterPtr(new ISM::APORater(votingSpaceInEval->binSize, votingSpaceInEval->maxProjectionAngleDeviation));
	break;
      default:
	this->mRater = ISM::RaterPtr(new ISM::SimpleRater(votingSpaceInEval->binSize, votingSpaceInEval->maxProjectionAngleDeviation));
	break;
      }

  }

  bool VotingResultCalculator::computeVotingResult(VotingResultPtr& result, TypeToInnerMap votedReferencePoses)

  {
    //We check if there are any reference object votes in the vote set from radius search.
    bool voteFromReferenceObjectExists;

	//Now we go through all votes, used as origins for fitting.
          for (TypeToInnerMap::iterator typeIt = votedReferencePoses.begin();
               typeIt != votedReferencePoses.end(); typeIt++)
          {
              for (IdToVoteMap::iterator idIt = typeIt->second.begin();
                   idIt != typeIt->second.end(); idIt++)
              {
                  //But we analyze this voxel grid element just once (one call by VotingSpace).
                  for (VotedPosePtrs::iterator voteIt = idIt->second.begin();
                       voteIt != idIt->second.end(); voteIt++)
                  {
		      //All objects are allowed to vote for this new originForFitting
                      sourcesWithFittingVotes.clear();

                      originForFitting = *voteIt;

		      //If there is any votedPose for the current type and id combination that we previously set as originForFitting it should not be at the same pose as that we now want to set as originForFitting.
		      if(voteIt != idIt->second.begin())
			{
			  VotedPosePtrs::iterator previousVoteIt = std::prev(voteIt);
			  if (GH::poseEqual((*voteIt)->pose, (*previousVoteIt)->pose))
			    continue;
			}

                      //Get votes to fit from radius search
                      TypeToInnerMap votes = mVotingSpaceInEval->collectVotesInSphere(originForFitting->pose->point, voteFromReferenceObjectExists);

                      //In case a reference vote exists in our votedPose, we just accept selfvotes from it as originForFitting.
                      if (enabledSelfVoteCheck && voteFromReferenceObjectExists && !GH::isSelfVote(originForFitting->vote))
                      {
                          continue;
                      }

                      //Set votedPose, to which others are going to be fitted, before we start matching the rest.
                      originPose = originForFitting->pose;
                      originType = typeIt->first;
                      originId = idIt->first;

                      summarizedFittingVotes = SummarizedVotedPosePtrs();

                      //originForFitting is trivially fitting itself and until now we have one exemplary of it.
                      summarizedFittingVotes.push_back(std::make_pair(originForFitting, std::make_pair(1u, originForFitting->weight)));

                      //Here we look for all fitting votes (from origin source and other objects) for the currently chosen origin.
                      searchFittingVotes(votes);

                      double confidence = 0;
                      ObjectSetPtr objSet(new ObjectSet());

		      //Create objects in RecognitionResult and calculate confidence of RecognitionResult.
                      for (SummarizedVotedPosePtr summarizedVote : summarizedFittingVotes)
                      {
                          VotedPosePtr votedPose = summarizedVote.first;
                          double tmpWeight = summarizedVote.second.second;
                          confidence += (tmpWeight / votedPose->weightDenominator);
                          ObjectPtr obj(new Object((*votedPose->source)));
                          obj->type = votedPose->vote->objectType;
                          obj->observedId = votedPose->vote->observedId;
                          obj->weight = tmpWeight;
                          if(obj->type.find("_sub") == std::string::npos)
                          {
                              obj->confidence = votedPose->weight;
                          }
                          objSet->insert(obj);
                      }
                      //We overwrite previous VotingResult for this votes if it is the best rated result.
                      if ((confidence > 0) && (!result || (result->confidence < confidence)))
                      {
                        result = VotingResultPtr(new VotingResult(objSet, originPose, confidence, summarizedFittingVotes));
			//Abort searching for this vote set once a perfect VotingResult for this bin, since we return one at all.
			if (confidence >= 1.0 - VotingSpace::epsilon)
			    return true;
                      }
                  }
              }
          }

        return (result != nullptr);

  }

  void VotingResultCalculator::searchFittingVotes(TypeToInnerMap votes)
  {

      //Count fitting votes for current type and id combination.
      unsigned int fittingVoteCounter;
      //For the current type and id combination, we explicitly just add the votedPose that fits best to originForFitting.  
      VotedPosePtr bestVotedPose;
      double bestWeight;

      //Fit votes for each combination of type and id.
      for (TypeToInnerMap::iterator typeIt = votes.begin();
           typeIt != votes.end(); typeIt++)
      {

          for (IdToVoteMap::iterator idIt = typeIt->second.begin();
               idIt != typeIt->second.end(); idIt++)
          {

	      //If we encounter votes from originForFitting, just count them if they are supporting.
	      if (typeIt->first == originType && idIt->first == originId)
		{
		  fittingVoteCounter = countSupportingOriginForFittingVotes(idIt->second);
		}
	      else
		{
		  countSupportingOriginForFittingVotes(idIt->second, fittingVoteCounter, bestWeight, bestVotedPose);
		}

	      //If we found any vote from the current type and id combination
              if (fittingVoteCounter > 0)
              {
		//We also count all fitting votes from the object that delivers originForFitting.
                  if(typeIt->first == originType && idIt->first == originId)
                  {
                      summarizedFittingVotes.front().second.first = fittingVoteCounter;
                      sourcesWithFittingVotes.insert(summarizedFittingVotes.front().first->source);
                  }
		  //Just add new optimally fitting votedPose to RecognitionResult if votedPos is not coming from object that delivers originForFitting.
                  else
                  {
                      summarizedFittingVotes.push_back(std::make_pair(bestVotedPose, std::make_pair(fittingVoteCounter, bestWeight)));
                      //Do not process votes from currently processed "object" for this originForFitting.
                      sourcesWithFittingVotes.insert(bestVotedPose->source);
                  }
              }
          }
      }
  }

}


/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Global includes
#include <map>
#include <set>
#include <vector>
#include <list>
#include <iostream>
#include <sstream>

//Local includes
#include "Recognizer.hpp"
#include "utility/GeometryHelper.hpp"
#include "VotedPose.hpp"
#include "VotingSpace.hpp"
#include "VotingResultCalculator.hpp"

#define SAME_REF_AND_RECOG_RESULT_CONFIDENCE_THRESH 0.01 

namespace ISM {

  Recognizer::Recognizer(const std::string& dbfilename, double bin_size, double maxProjectionAngleDeviation, int raterType) :
    bin_size(bin_size), maxProjectionAngleDeviation(maxProjectionAngleDeviation), mRaterType(raterType)
  {
    this->tableHelper = TableHelperPtr(new TableHelper(dbfilename));
    this->objectTypes = this->tableHelper->getObjectTypes();
    this->voteSpecifiersPerObject = this->tableHelper->getVoteSpecifiersForObjectTypes(this->objectTypes);
    this->getPatternDefinitions();
    this->arrangePatternsAccordingToTreeHeight();
  }

  const std::vector<RecognitionResultPtr> Recognizer::recognizePattern(const ObjectSetPtr& objectSet,
								       const double filterThreshold, const int resultsPerPattern) {
    this->inputSet = ObjectSetPtr( new ObjectSet(*objectSet));
    this->votingCache.clear();
    this->ismResults.clear();

    //Scene recognition is an iterative process in which isms in trees are evaluated according to height tree height.
    TreeHeightToPatternName::reverse_iterator treeHeightIt;
    //Trees are evaluated starting at the leaf with max tree height in the db, going until the roots of the trees.
    for(treeHeightIt = patternPerTreeHeight.rbegin(); treeHeightIt != patternPerTreeHeight.rend(); treeHeightIt++) {

      //Get voted poses (also from cache) for all objects in all isms.
      this->calculateVotedPosesForAllObjects();
      //Search for RecognitionResults in all sub-ISMs at the current tree height
      this->fillAndEvalVotingSpaceAtTreeHeight(treeHeightIt->first);

    }
    
    //Build up scene recognition results for ism trees from RecognitionResults of single sub-ISMs in tree.
    return this->assembleIsmTrees(this->ismResults, filterThreshold, resultsPerPattern);
  
  }

  void Recognizer::calculateVotedPosesForAllObjects()
  {

    //Delete old voting results from last iteration step, since pattern content might have changed.
    patternToVotedPoses.clear();

    //Start calculating reference pose votes by going through all object estimated we got as input.
    for (ObjectPtr& object : this->inputSet->objects) {

      //Just copy VotedPoses from voting cache if already existing for current object.
      std::map<ObjectPtr, std::vector<VotedPosePtr> >::iterator cacheIt = votingCache.find(object);
      if (cacheIt != votingCache.end()) {
	std::vector<VotedPosePtr> votedPoses = (*cacheIt).second;
	for (VotedPosePtr& v : votedPoses) {
	  std::string patternName = v->vote->patternName;
	  PatternNameToVotedPoses::iterator vmit = patternToVotedPoses.find(patternName);
	  if (vmit == patternToVotedPoses.end()) {
	    vmit = patternToVotedPoses.insert(std::make_pair(patternName, std::vector<VotedPosePtr>())).first;
	  }
	  (*vmit).second.push_back(v);
	}
	continue;
      }

      //When object has no type (ignore_type==true), it votes for all object types.
      std::set<std::string> types;
      if (object->type == "")
        {
	  types = this->objectTypes;
        }
      //Otherwise just for its own type.
      else
        {
	  types.insert(object->type);
        }

      //Match input object estimates to votes from ism table according to type and id information.
      for (const std::string& objectType : types)
        {
	  //Get all vote specifiers for that object type.
	  std::vector<VoteSpecifierPtr> votes = this->voteSpecifiersPerObject[objectType];
	  for (VoteSpecifierPtr& vote : votes)
            {
	      //Just use vote specifiers for matching ids or all votes for that type in case ignore_ids==true.
	      if (object->observedId == "" || object->observedId == vote->observedId)
                {

		  PatternPtr pattern = this->patternDefinitions[vote->patternName];
		  //Calculate reference pose vote for given combination of input object estimate and vote (relative pose).
		  PosePtr pose = this->calculatePoseFromVote(object->pose, vote);
		  if (patternToVotedPoses.find(vote->patternName) == patternToVotedPoses.end())
                    {
		      patternToVotedPoses[vote->patternName] = std::vector<VotedPosePtr>();
                    }
		  VotedPosePtr v(new VotedPose(pose, vote, object, object->weight, pattern->expectedMaxWeight));
		  patternToVotedPoses[vote->patternName].push_back(v);

		  //Fill voting cache for the following iterations in iterative scene recognition.
		  std::map<ObjectPtr, std::vector<VotedPosePtr> >::iterator cacheIt = votingCache.find(object);
		  if (cacheIt == votingCache.end()) {
		    cacheIt = votingCache.insert(std::make_pair(object, std::vector<VotedPosePtr>())).first;
		  }

		  (*cacheIt).second.push_back(v);

                }
            }
        }
    }   
  }

  void Recognizer::fillAndEvalVotingSpaceAtTreeHeight(unsigned int treeHeight)
  {

    VotingSpacePtr vsPtr = VotingSpacePtr(new VotingSpace(this->bin_size, this->maxProjectionAngleDeviation, this->mRaterType));

    //Each ISM in a tree is treated as a separate pattern. Fetching all isms for all trees in the db at a given height.
    for (std::string patternName : patternPerTreeHeight.at(treeHeight)) {

      PatternNameToVotedPoses::iterator votedPosesForCurrentPattern = patternToVotedPoses.find(patternName);
      //We can only evaluate patterns for which object estimates have been acquired.
      if(votedPosesForCurrentPattern == patternToVotedPoses.end())
	continue;

      //patternToVotedPoses is a container for voted poses (reference pose estimates) of all isms in the db.
      //Insert all votedPoses (reference pose votes) into voxelgrid to discretize search space.
      VotingResultPtrs vsresults = vsPtr->fillAndEvalVotingSpace(votedPosesForCurrentPattern->second);

      // Buffer VotingSpace for its visualization if request exists.
      if (!patternForVotingSpaceViz.empty() && (patternName.find(patternForVotingSpaceViz) == 0))
        {

	  votingSpaceBuffer = std::make_pair(patternName, vsPtr);

	  //Create a second voting space to revent the first (which is to be visualized) from being cleared for the next considered ISM.
	  vsPtr = VotingSpacePtr(new VotingSpace(this->bin_size, this->maxProjectionAngleDeviation, this->mRaterType));
        }

      //Convert results from each voxel grid element into single Recognition result from sub-ISM and extend input object set.
      for (VotingResultPtr& vsres : vsresults) {
	RecognitionResultPtr res(new RecognitionResult(patternName, vsres->referencePose,
						       vsres->matchingObjects, vsres->confidence,
						       vsres->summarizedVotes));

	//Replace recognition results of sub-ISMs at same reference-pose if their rating improved during this step in the loop.
	int resIdx = this->resultAlreadyExisting(res);

	if(resIdx >= 0) {
	  if(this->ismResults[resIdx]->confidence >= res->confidence)
	    continue;
	  else
	    this->ismResults[resIdx] = res;
	}

	else
	  this->ismResults.push_back(res);

	//Insert RecognitionResults of sub-ISMs in tree into input object set to propagate ratings though the tree (preparation of next voting iteration).
	if (this->voteSpecifiersPerObject.find(res->patternName) != this->voteSpecifiersPerObject.end()) {
	  // if we have an object on record with a name that matches a pattern name
	  // treat the referencePose as a new object and repeat the recognition to capture subscenes
	  ObjectPtr refObj(new Object(res->patternName, res->referencePose));
	  //Reference object confidence always corresponds to the confidence of the corresponding recognition result.
	  refObj->confidence = vsres->confidence;
	  //weight is the unnormalized confidence of this recognition result and used to calculate the confidence of potential recognition results in the next-heigher ism.
	  double weightSum = 0;
	  for (ObjectPtr& obj : vsres->matchingObjects->objects)
	    {
	      weightSum += obj->weight;
	    }
	  refObj->weight = weightSum;
	  //Look if we add a new object to the input object set or if we might replace an object estimate by an instance with a better confidence.
	  int inSetIdx = this->objectAlreadyInSet(refObj);
	  if (inSetIdx >= 0) {
	    ObjectPtr setObj = this->inputSet->objects[inSetIdx];
	    if (setObj->confidence < refObj->confidence || setObj->weight
		< refObj->weight) {

	      this->inputSet->objects.erase(this->inputSet->objects.begin() + inSetIdx);
	      inSetIdx = -1;
	    }
	  }
	  if (inSetIdx < 0)
	    this->inputSet->insert(refObj);

	}         
      }
    }

  }

  int Recognizer::resultAlreadyExisting(const RecognitionResultPtr& res) {
    for (size_t i = 0; i < this->ismResults.size(); i++)
      {
        if (this->ismResults[i]->patternName == res->patternName
            && GeometryHelper::poseEqual(this->ismResults[i]->referencePose, res->referencePose))
	  {
            return i;
	  }
      }
    return -1;
  }

  int Recognizer::objectAlreadyInSet(const ObjectPtr& o) {
    for (size_t i = 0; i < this->inputSet->objects.size(); i++) {
      ISM::ObjectPtr setObj = this->inputSet->objects[i];
      if (setObj->type == o->type && setObj->observedId == o->observedId
	  && GeometryHelper::poseEqual(setObj->pose, o->pose)) {
	return i;
      }
    }
    return -1;
  }

  PosePtr Recognizer::calculatePoseFromVote(const PosePtr& pose, const VoteSpecifierPtr& vote) const {
    PointPtr referencePoint = GeometryHelper::applyQuatAndRadiusToPose(pose, vote->objectToRefQuat, vote->radius);
    return GeometryHelper::getReferencePose(pose, referencePoint, vote->objectToRefPoseQuat);
  }

  void Recognizer::getPatternDefinitions() {

    if(this->voteSpecifiersPerObject.empty())
      this->voteSpecifiersPerObject = this->tableHelper->getVoteSpecifiersForObjectTypes(this->objectTypes);

    typedef std::pair<std::string, std::vector<VoteSpecifierPtr> > mapItemType;
    std::set<std::string> patternNames;
    for(mapItemType item : this->voteSpecifiersPerObject) {
      for(VoteSpecifierPtr vote : item.second) {
	patternNames.insert(vote->patternName);
      }
    }
    this->patternDefinitions = this->tableHelper->getPatternDefinitionsByName(patternNames);
  }

  void Recognizer::arrangePatternsAccordingToTreeHeight() {

    if(this->voteSpecifiersPerObject.empty())
      this->voteSpecifiersPerObject = this->tableHelper->getVoteSpecifiersForObjectTypes(this->objectTypes);

    if(!this->patternPerTreeHeight.empty())
      this->patternPerTreeHeight.clear();

    //Using just a set over object types since subpatterns have no id.
    std::map<std::string, std::set<std::string> > patternToSubpatterns;
    //All ISM that are at the roots of the ISM trees in the database.
    std::vector<std::string> topLevelPatterns;

    //Construct mapping from pattern to subpatterns for each ISM in database.
    typedef std::pair<std::string, std::vector<VoteSpecifierPtr> > mapItemType;
    for(mapItemType item : this->voteSpecifiersPerObject) {
      for(VoteSpecifierPtr vote : item.second) {

	std::map<std::string, std::set<std::string> >::iterator patternIt;
	patternIt = patternToSubpatterns.find(vote->patternName);

	//Add additional subpattern to already known pattern.
	if(patternIt != patternToSubpatterns.end()){
	  //The object voting is no subpattern of any ism, so we add nothing.
	  if(vote->objectType.find("_sub") == std::string::npos)
	    continue;
	  else
	    patternIt->second.insert(vote->objectType);
	}
	//Create a new mapping from pattern to subpattern with the current <pattern, subpattern> pair.
	else{
	  std::set<std::string> newSubpatterns;
	  //Add an empty set, if we encounter a real object instead of a subpattern.
	  if(vote->objectType.find("_sub") != std::string::npos)
	    newSubpatterns.insert(vote->objectType);
	  patternToSubpatterns.insert(std::pair<std::string, std::set<std::string> >(vote->patternName, newSubpatterns));
	}

      }
    }

    //Go through all mappings from pattern to subpatterns to find root patterns in db.
    for(auto& valuePair : patternToSubpatterns)      
      if (valuePair.first.find("_sub") == std::string::npos)
	topLevelPatterns.push_back(valuePair.first);

    //Pattern which are to be inserted in the height to patternMapping
    std::list<std::pair<std::string, unsigned int> > breadthFirstPatternQueue;

    //First all top level ism need to be processed.
    for(std::string currentTopLevelPattern : topLevelPatterns)
      breadthFirstPatternQueue.push_back(std::pair<std::string, unsigned int>(currentTopLevelPattern, 0));

    //Bread first search on pattern mappings to arrange patterns according to their height in any of the ISM trees in the db.
    while(breadthFirstPatternQueue.size()) {

      //Get next item to be added to height mapping.
      std::pair<std::string, unsigned int> patternAtHeight = breadthFirstPatternQueue.front();
      //Delete item from queue that going to be added to the height mapping if not already done.
      breadthFirstPatternQueue.pop_front();

      TreeHeightToPatternName::iterator heightMappingIt;

      //Are there any mappings for the height of the current pattern?
      heightMappingIt = patternPerTreeHeight.find(patternAtHeight.second);
      if(heightMappingIt != patternPerTreeHeight.end())
	//Height already known in mapping, just adding Pattern.
	heightMappingIt->second.insert(patternAtHeight.first);
      //Create mapping from height to pattern.
      else{

	std::set<std::string> newPattern;
	newPattern.insert(patternAtHeight.first);
	patternPerTreeHeight.insert(std::pair<unsigned int,std::set<std::string> >(patternAtHeight.second, newPattern));

      }

      //Get subpatterns of pattern that we added to height mapping.
      std::map<std::string, std::set<std::string> >::iterator currentSubPatterns;
      currentSubPatterns = patternToSubpatterns.find(patternAtHeight.first);
      //Add all subpatterns to queue.
      for(std::string subPattern : currentSubPatterns->second){
	  
	//Add subpattern of current pattern to queue with its height.
	breadthFirstPatternQueue.push_back(std::pair<std::string, unsigned int>(subPattern, patternAtHeight.second+1));

      }

    }

  }

  std::vector<RecognitionResultPtr> Recognizer::assembleIsmTrees(const std::vector<RecognitionResultPtr>& ismResults,
								 const double filterThreshold, const int resultsPerPattern) {
    std::map<std::string, std::vector<RecognitionResultPtr> > patternNameToResults;

    //Find root ism recognition results among recognition results for all isms in ism trees in database.
    std::vector<RecognitionResultPtr> topLevelResults;
    for (const RecognitionResultPtr& res : ismResults) {
      if (res->patternName.find("_sub") == std::string::npos) {
	topLevelResults.push_back(res);
      }
      //Create map keys for all top-level pattern names present in recognition results.
      std::map<std::string, std::vector<RecognitionResultPtr> >::iterator it = patternNameToResults.find(res->patternName);
      if (it == patternNameToResults.end())
        {
	  it = patternNameToResults.insert(std::make_pair(res->patternName, std::vector<RecognitionResultPtr>())).first;
        }
      (*it).second.push_back(res);
    }

    //Sort recognition results for root isms according to patternName.
    std::map<std::string, std::vector<RecognitionResultPtr> > resultsForPattern;
    for (RecognitionResultPtr& res : topLevelResults) {
      std::map<std::string, std::vector<RecognitionResultPtr> >::iterator it = resultsForPattern.find(res->patternName);
      if (it == resultsForPattern.end()) {
	std::vector<RecognitionResultPtr> p;
	p.push_back(res);
	resultsForPattern.insert(std::make_pair(res->patternName, p));
      } else {
	(*it).second.push_back(res);
      }
    }
    std::vector<RecognitionResultPtr> ret;

    //sort results by confidence and gather the subpatterns for the requested number of results per pattern
    for (const std::pair<std::string, std::vector<RecognitionResultPtr> >& patternPair : resultsForPattern) {
      std::vector<RecognitionResultPtr> rlist = patternPair.second;
      //Sort results for a given pattern.
      std::stable_sort(rlist.begin(), rlist.end(), [](const RecognitionResultPtr& o1, const RecognitionResultPtr& o2) {
	  return o1->confidence > o2->confidence;
        });
      //Make sure to only take the "n"-best recognition results for this pattern.
      for (size_t i = 0; i < rlist.size() && (resultsPerPattern < 0 || (int)i < resultsPerPattern); i++) {
	if (rlist[i]->confidence >= filterThreshold) {
	  RecognitionResultPtr r = rlist[i];
	  ret.push_back(r);
	  //Hang subpattern under currently considered recognition result of root ism.
	  r->subPatterns = Recognizer::getSubPatternsForResult(r, patternNameToResults);
	} else {
	  break;
	}
      }
    }
    //Resort list of best recognition results for all patterns.
    std::stable_sort(ret.begin(), ret.end(), [](const RecognitionResultPtr& o1, const RecognitionResultPtr& o2) {
        return o1->confidence > o2->confidence;
      });

    return ret;

  }

  std::vector<RecognitionResultPtr> Recognizer::getSubPatternsForResult(RecognitionResultPtr result,
									std::map<std::string, std::vector<RecognitionResultPtr> > patternNameToResults) {
    std::vector<RecognitionResultPtr> ret;
    for (ObjectPtr& obj : result->recognizedSet->objects) {
      std::map<std::string, std::vector<RecognitionResultPtr> >::iterator it = patternNameToResults.find(obj->type);
      if (it == patternNameToResults.end()) {
	continue;
      }
      for (RecognitionResultPtr& originalResult : (*it).second) {
	PosePtr originalPose = originalResult->referencePose;
	PosePtr currentPose = obj->pose;
	//We link isms by searching pairs of reference objects and recognition results with identical poses and confidences.
	if (GeometryHelper::poseEqual(currentPose, originalPose)
	    && fabs(obj->confidence - originalResult->confidence) < SAME_REF_AND_RECOG_RESULT_CONFIDENCE_THRESH) {
	  ret.push_back(originalResult);
	  break;
	}
      }
    }
    for (RecognitionResultPtr& subPattern : ret) {
      subPattern->subPatterns = Recognizer::getSubPatternsForResult(subPattern, patternNameToResults);
    }
    return ret;
  }

}



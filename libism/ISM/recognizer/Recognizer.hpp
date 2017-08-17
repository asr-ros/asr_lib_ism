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

//Global includes
#include <string>
#include <boost/shared_ptr.hpp>

//Local includes
#include "common_type/ObjectSet.hpp"
#include "utility/TableHelper.hpp"
#include "common_type/RecognitionResult.hpp"
#include "VotedPose.hpp"
#include "typedef.hpp"

namespace ISM {
  /**
   * Recognizer class. Class in which scene recognition with (non-)hierarchical isms is performed based on a hough voting scheme and vote backprojection. See Meissner et al. 2013 in Section IV and V (B).
   */
  class Recognizer {

    //Accessor to db, containing isms.
    TableHelperPtr tableHelper;
    //Object constellation in which scene is to be recognized.
    ObjectSetPtr inputSet;

    //Parameters of voting space.
    double bin_size;
    double maxProjectionAngleDeviation;

    //All types of objects in sqlite table we got as input (for all scenes in this table)
    std::set<std::string> objectTypes;

    //Votes for all objects in db, extracted from db and held in memory to calculate votesPoses.
    ObjectTypeToVoteMap voteSpecifiersPerObject;
    //Used to get max expected weight per pattern.
    PatternNameToPatternMap patternDefinitions;

    //All patterns in db, arranged according to their height.
    TreeHeightToPatternName patternPerTreeHeight;
    //All voted poses, calculated during voting, arranged according to the pattern, in which voxel grid evaluation should take place.
    PatternNameToVotedPoses patternToVotedPoses;

    bool enabledSelfVoteCheck;

    //Simple or APO-Rater enum.
    const int mRaterType;

    std::vector<RecognitionResultPtr> ismResults;
    //Required to ensure that objects only vote once since, object voting and ism evaluation throughout the tree is decoupled.
    std::map<ObjectPtr, std::vector<VotedPosePtr> > votingCache;

    std::string patternForVotingSpaceViz;
    //Copy of a voting space, filled by an ism, for its visualization.
    PatternNameAndVotingSpaceTuple votingSpaceBuffer;

  public:
    /**
     * Create recognition interface to an sqlite db.
     *
     * @param dbfilename File containing scenes we want to be able to detect with this recognizer.
     * @param bin_size Side length of cubes (bins) making up voxel grid in which hough voting is performed. Maximal accepted distance between scene reference hypotheses of different objects in a bin.
     * @param maxProjectionAngleDeviation Maximal accepted difference in orientations of scene reference hypotheses of different objects in a bin.
     * @param enabledSelfVoteCheck Self-Vote-Check: Ensures the use of votedPose from a selfvoting object as originForFitting, if such an object exist.
     * @param raterType Objective function that is used for rating how well votedPoses in voxel grid match each other (especially in recognition results).
     */
    Recognizer(const std::string& dbfilename, double bin_size, double maxProjectionAngleDeviation, bool enabledSelfVoteCheck, int raterType = 0);

    /**
     * Find instances of scenes (or scene models) loaded beforehand in a spatial configuration of objects provided to this method. Returns hypotheses on existing scenes as subsets of that configuration that match a model during scene recognition by our hough voting scheme. Hyptheses are rated by confidences being the percentage of objects in the scene we detected in the provided object configuration.
     *
     * @param objectSet Object configuration in which scenes are tried to be detected.
     * @param filterThreshold Minimum confidence that a recognition result must dispose of, to be returned.
     * @param resultsPerPattern Maximum number of recognition results for each scene in db loaded beforehand.
     * @param patternName name of pattern which should be recognized.
     * @return Recognition results in decreasing order of confidence.
     */
    const std::vector<RecognitionResultPtr> recognizePattern(const ObjectSetPtr& objectSet,
                                 const double filterThreshold = 0.0, const int resultsPerPattern = -1, const std::string targetPatternName = "");
    
    std::map<ObjectPtr, std::vector<VotedPosePtr> > getVotingCache()
    {
      return votingCache;
    }

    //Interfaces to change settings of recognizer during runtime.

    void setVoteSpecifiersPerObject(ISM::ObjectTypeToVoteMap voteSpecifiersPerObject)
    {
      this->voteSpecifiersPerObject = voteSpecifiersPerObject;
    }

    void setObjectTypes(std::set<std::string> objectTypes)
    {
      this->objectTypes = objectTypes;
    }

    void setPatternDefinitions(ISM::PatternNameToPatternMap patternDefinitions)
    {
      this->patternDefinitions = patternDefinitions;
    }

    //Create mapping that assigns all isms in the trees in db to their height in their tree.
    void arrangePatternsAccordingToTreeHeight();
	
    //Reset everything before looking for another scene

    void clearData()
    {
      objectTypes.clear();
      assert(objectTypes.size() == 0);
      voteSpecifiersPerObject.clear();
      assert(voteSpecifiersPerObject.size() == 0);
      patternDefinitions.clear();
      assert(patternDefinitions.size() == 0);
    }

    //Interfaces for ISM voting visualization

    void setVotingSpaceToBeBuffered(std::string patternName)
    {
      this->patternForVotingSpaceViz = patternName;
    }

    PatternNameAndVotingSpaceTuple getBufferedVotingSpace()
    {
      this->patternForVotingSpaceViz.clear();
      return votingSpaceBuffer;
    }

  private:

    //Two methods that make up iteration step of iterative scene recognition process.

    //Calculates voted poses (reference pose hypotheses), using resp. filling the voting cache, and assigns them to a pattern.
    void calculateVotedPosesForAllObjects();
    //For all ISMs at a given tree height: Inserts all voted poses into voxel grids and evaluates each element of the grid.
    void fillAndEvalVotingSpaceAtTreeHeight(unsigned int treeHeight);

    int resultAlreadyExisting(const RecognitionResultPtr& res);
    int objectAlreadyInSet(const ObjectPtr& o);
    PosePtr calculatePoseFromVote(const PosePtr& pose, const VoteSpecifierPtr& vote) const;

    //Create mapping from patternname String to patterns in db.
    void getPatternDefinitions();

    //After having performed iterative scene recognition process, gather tree like scene recognition result from the single RecognitionResults from all sub-ISMs in the ISM tree. Uses getSubPatternsForResult.
    static std::vector<RecognitionResultPtr> assembleIsmTrees(const std::vector<RecognitionResultPtr>& ismResults,
                                  const double filterThreshold, const int resultsPerPattern, const std::string targetPatternName);

    //Search matching RecognitionResults in child ISMs in tree to build up tree structure of scene recognition result.
    static std::vector<RecognitionResultPtr> getSubPatternsForResult(RecognitionResultPtr result,
								     std::map<std::string, std::vector<RecognitionResultPtr> > patternNameToResults);

  };
  typedef boost::shared_ptr<Recognizer> RecognizerPtr;

}



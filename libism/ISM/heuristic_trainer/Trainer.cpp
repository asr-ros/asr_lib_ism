/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <iostream>
#include <set>
#include <math.h>

#include "Trainer.hpp"
#include "common_type/RecordedPattern.hpp"
#include "utility/GeometryHelper.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "common_type/Pose.hpp"
#include "common_type/Tracks.hpp"
#include "StaticRelationHeuristic.hpp"
#include "DirectionRelationHeuristic.hpp"
#include "DirectionOrientationRelationHeuristic.hpp"
#include "DataCollector.hpp"

namespace ISM {
    Trainer::Trainer(std::string dbfilename, bool dropOldModelTables) : mUseManualDefHeuristic(false),
    		mUsePredefinedRefs(false),
    		staticBreakRatio(0.0), togetherRatio(0.0), maxAngleDeviation(0.0)
    {
        this->tableHelper.reset(new TableHelper(dbfilename));
        if(dropOldModelTables)
        {
        	tableHelper->createTablesIfNecessary();
        	tableHelper->dropModelTables();
        	tableHelper->createTablesIfNecessary();
        }
        this->skips = 0;
        this->useClustering = true;
    }

    void Trainer::setSkipsPerCycle(int skips) {
        this->skips = skips;
    }

    void Trainer::setUseClustering(bool useClustering) {
        this->useClustering = useClustering;
    }

    void Trainer::trainPattern() {
        std::vector<std::string> patternNames = this->tableHelper->getRecordedPatternNames();
        std::cerr<<"found "<<patternNames.size()<<" patterns"<<std::endl;  
    for (std::string& name : patternNames) {
            this->trainPattern(name);
	}
    }

    void Trainer::trainPattern(const std::string& patternName) {
        boost::shared_ptr<RecordedPattern> r = this->tableHelper->getRecordedPattern(patternName);
        if (!r) {
            std::cerr<<"no pattern records found for pattern "<<patternName<<std::endl;
        } else {
            std::cerr<<"training "<<patternName<<std::endl;
            this->recordedPattern = r;
            this->learn();
        }
    }

    void Trainer::learn() {
        std::vector<ObjectSetPtr> sets = this->recordedPattern->objectSets;
        for (size_t i = 0; i < sets.size(); i++) {
	  if (sets[i]->objects.size() == 0) {
                sets.erase(sets.begin() + i);
                i--;
            }
        }

        int clusterId = 0;
        TracksPtr tracks(new Tracks(sets));

        while (this->useClustering) {
            HeuristicPtr heuristic = this->findHeuristicMatch(tracks);
            if (!heuristic || (int) tracks->tracks.size() <= 2
            		|| heuristic->cluster->tracks.size() == tracks->tracks.size()) {
                break;
            }

            TracksPtr cluster = heuristic->cluster;
            std::stringstream subPatternNameStream;
            if(heuristic->clusterId < 0)
            {
                subPatternNameStream<<this->recordedPattern->name<<"_sub"<<clusterId;
            } else
            {
                subPatternNameStream<<this->recordedPattern->name<<"_sub"<<heuristic->clusterId;
                clusterId = std::max(clusterId, heuristic->clusterId);
            }
            clusterId++;
            std::string subPatternName = subPatternNameStream.str();

            TrackPtr refTrack = this->doTraining(cluster->toObjectSetVector(), subPatternName);
            if (DataCollector::shouldCollect()) {
                DataCollector::getData()->tracksWithRef.push_back(TracksWithRef(cluster, refTrack));
            }
            tracks->replace(cluster->tracks, refTrack);
        }

        //train remaining sets
        TrackPtr refTrack = this->doTraining(tracks->toObjectSetVector(), this->recordedPattern->name);
        if (DataCollector::shouldCollect()) {
            DataCollector::getData()->tracksWithRef.push_back(TracksWithRef(tracks, refTrack));
        }
    }

    void Trainer::setClusterForManualDefHeuristic(std::vector<std::pair<std::vector<
    		ManuallyDefPseudoHeuristic::ClusterObject>, uint16_t>> cluster)
    {
    	mClusterForManualDefHeuristic = cluster;
    	mUseManualDefHeuristic = true;
    }
    void Trainer::setPredefinedRefs(std::map<std::string, std::string>& refs)
    {
    	mPatternToTypesOfPredefinedRefs = refs;
    	mUsePredefinedRefs = true;
    }
    HeuristicPtr Trainer::findHeuristicMatch(const TracksPtr& tracks) {
        HeuristicPtr bestHeuristic;

        std::vector<HeuristicPtr> heuristics;
        if(mUseManualDefHeuristic)
        {
        	heuristics.push_back(HeuristicPtr(new ManuallyDefPseudoHeuristic(mClusterForManualDefHeuristic)));
        }
        heuristics.push_back(HeuristicPtr(new DirectionRelationHeuristic(staticBreakRatio, togetherRatio, maxAngleDeviation)));
        //heuristics.push_back(HeuristicPtr(new DirectionOrientationRelationHeuristic(staticBreakRatio, togetherRatio, maxAngleDeviation)));
        for (HeuristicPtr& heuristic : heuristics) {
	  
	    heuristic->applyHeuristic(tracks);

            if (!heuristic->cluster) {
                continue;
            }
            std::cerr<<"heuristic results of "<<heuristic->name<<std::endl;
            std::cerr<<heuristic->cluster->tracks.size()<<" tracks, confidence: "<<heuristic->confidence<<std::endl;
            if (heuristic->confidence > 0.7 && (!bestHeuristic || heuristic->confidence > bestHeuristic->confidence)) {
                bestHeuristic = heuristic;
            }
        }

        return bestHeuristic ? bestHeuristic : HeuristicPtr();
    }

    TrackPtr Trainer::doTraining(std::vector<ObjectSetPtr> sets, std::string patternName) {
        int toSkip = 0;
        int setCount = 0;
        double objectsWeightSum = 0;
        std::string refType = "";
        std::string refId = "";

        TrackPtr refTrack(new Track(patternName));

        TracksPtr tracks(new Tracks(sets));

        bool refFound = false;
        if(mUsePredefinedRefs)
        {
        	if(mPatternToTypesOfPredefinedRefs.find(patternName) != mPatternToTypesOfPredefinedRefs.end())
        	{
                for (std::vector<TrackPtr>::iterator track = tracks->tracks.begin(); track != tracks->tracks.end();
			    		++track)
			    {
                    for (ObjectPtr& obj : (*track)->objects) {
					    if (obj) {
                            if(obj->type == mPatternToTypesOfPredefinedRefs[patternName])
					    	{
                                refType = obj->type;
                                refId = obj->observedId;
					    		refFound = true;
					    	} else
					    	{
					    		break;
					    	}
					    }
					}
			    }
        	}
        }
        if(!refFound)
        {
            double bestViewRatio = 0;
            double bestMovement = 0;
            for (TrackPtr& track : tracks->tracks) {
                int views = 0;
                std::string refT;
                std::string refI;
                ObjectPtr lastObj;
                double movement = 0;
                for (ObjectPtr& obj : track->objects) {
                    if (obj) {
                        refT = obj->type;
                        refI = obj->observedId;
                        views++;
                        if (lastObj) {
              movement += GeometryHelper::getDistanceBetweenPoints(obj->pose->point, lastObj->pose->point);
                        }
                        lastObj = obj;
                    }
                }

                double ratio = (double)views / (double)track->objects.size();
                if (ratio > bestViewRatio || (ratio == bestViewRatio && movement < bestMovement)) {
                    refType = refT;
                    refId = refI;
                    bestViewRatio = ratio;
                    bestMovement = movement;
                }
            }
        }

        std::cerr<<"choose ref "<<refType<<" : "<<refId<<std::endl;
        std::cerr<<"training "<<patternName<<" "<<std::endl;

        for (size_t setIdx = 0; setIdx < sets.size(); setIdx++)
        {
            double setWeightSum = 0;
            if (toSkip == 0) {
                toSkip = this->skips;
                std::cerr<<".";
                std::cerr.flush();
            } else {
                std::cerr<<"_";
                std::cerr.flush();
                toSkip--;
                continue;
            }

            PosePtr referencePose;
            setCount++;
            std::vector<ObjectPtr> objects = sets[setIdx]->objects;

            for (ObjectPtr& o : objects) {
                if (o->type == refType && o->observedId == refId) {
                    referencePose.reset(new Pose(*(o->pose)));
                    break;
                }
            }
            if (!referencePose && objects.size() > 0) {
                referencePose.reset(new Pose(*(objects[0]->pose)));
            } else if (!referencePose) {
                refTrack->objects.push_back(ObjectPtr());
                continue;
            }

            for (ObjectPtr& o : objects) {
                VoteSpecifierPtr vote = GeometryHelper::createVoteSpecifier(o->pose, referencePose);
                vote->patternName = patternName;
                vote->observedId = o->observedId;
                vote->objectType = o->type;
                vote->trackIndex = setIdx;
                this->tableHelper->insertModelVoteSpecifier(vote);
                setWeightSum += o->weight;
            }

            ObjectPtr refObj = ObjectPtr(
                new Object(
                    patternName,
                    referencePose
                )
            );

            refObj->weight = setWeightSum;

            refTrack->objects.push_back(refObj);

            objectsWeightSum += setWeightSum;

        }

        this->tableHelper->upsertModelPattern(
            patternName,
            floor(((float)objectsWeightSum / (float)setCount) + 0.5)
        );

        refTrack->calculateWeight();

        std::cerr<<"done ("<<refTrack->weight<<")"<<std::endl;

        return refTrack;
    }
}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Tree.hpp"
#include "utility/Util.hpp"


namespace ISM
{

	Tree::Tree(std::string pattern, unsigned clusterId, const TrackPtr reference,
			std::vector<ClusterPtr> &clustersLeft, const std::vector<ClusterPtr> &allClusters,
			IsmPtr &ism, bool naive)
	{
		build(pattern, clusterId, reference, clustersLeft, allClusters, ism, naive);
	}

	Tree::Tree(const std::string &pattern, ObjectRelations topology, bool naive)
	{
		LogHelper::logMessage("Generating Tree: ", LOG_INFO, LOG_COLOR_TREE);
		std::vector<ClusterPtr> clusters = buildClusters(topology);
		unsigned clusterId = 0;
		std::stringstream subPatternNameStream;
		subPatternNameStream<<pattern;
		std::string subPatternName = subPatternNameStream.str();
		IsmPtr ism (new ImplicitShapeModel());
        std::vector<ClusterPtr> copy = clusters;
        std::vector<ClusterPtr> allClusters = clusters;
		std::sort(clusters.begin(), clusters.end(), compareClusters);
		std::sort(allClusters.begin(), allClusters.end(), compareClusters);
        for (ClusterPtr& it : clusters)
		{
			LogHelper::logMessage("Recommended Height is " + std::to_string(it->getRecommendedLevel()), LOG_DEBUG, LOG_COLOR_TREE);
		}
		build(subPatternName, clusterId, (*clusters.begin())->getReference(), copy, allClusters, ism, naive);


		for (std::map<std::string, std::vector<VoteSpecifierPtr> >::iterator it = ism->voteSpecifiersPerObject.begin(); 
			 it != ism->voteSpecifiersPerObject.end();
			 ++it)
		{
			if (it->first.find("_sub") != std::string::npos)
			{
				for (unsigned int i = 0; i < it->second.size(); ++i)
				{
					it->second[i]->observedId = "";
				}
			}
		}

		this->setISM(ism);
		LogHelper::logMessage("Tree is generated\n", LOG_INFO, LOG_COLOR_TREE);
		LogHelper::logLine(LOG_DEBUG);
	}


	void Tree::build(std::string pattern, unsigned clusterId, const TrackPtr reference,
			std::vector<ClusterPtr>& clustersLeft, const std::vector<ClusterPtr>& allClusters,
			IsmPtr& ism, bool naive) {

		this->height = 0;
		this->weight = 1;
		this->reference = TrackPtr(new Track(*reference));

		ClusterPtr cluster;
		std::vector<ClusterPtr>::iterator clusterIt;
		for (clusterIt = clustersLeft.begin(); clusterIt != clustersLeft.end(); ++clusterIt)
		{
			if (reference->type == (*clusterIt)->getReference()->type &&
				reference->observedId == (*clusterIt)->getReference()->observedId)
			{
				cluster = *clusterIt;
				clustersLeft.erase(clusterIt);
				break;
			}
		}
		if (cluster != 0)
		{
			std::stringstream subPatternNameStream;
			if (pattern.find("_sub") != std::string::npos)
			{
				subPatternNameStream<<pattern<<clusterId;
				pattern = subPatternNameStream.str();
				this->getReference()->type = pattern;
				this->getReference()->observedId = "";
				this->setWeight(0);
				LogHelper::logMessage("Renamed to " + this->getReference()->type, LOG_DEBUG, LOG_COLOR_TREE);

				subPatternNameStream<<"_sub";
				pattern = subPatternNameStream.str();
			}
			else
			{
				this->getReference()->type = pattern;
				this->getReference()->observedId = "";
				this->setWeight(0);
				LogHelper::logMessage("Renamed to " + this->getReference()->type, LOG_DEBUG, LOG_COLOR_TREE);
				subPatternNameStream<<pattern<<"_sub";
				pattern = subPatternNameStream.str();
			}
            for (TrackPtr& child : cluster->getChildren()->tracks)
			{
				bool ignore = false;
				LogHelper::logMessage("naive is: " + std::to_string(naive), LOG_DEBUG, LOG_COLOR_TREE);
				if (naive == false)
				{
                    for (const ClusterPtr& clusterChecker : allClusters)
					{
						if (clusterChecker == cluster)
						{
							//Clusters are ordered by level they'd have in a tree
							break;
						}
                        for (TrackPtr& childChecker : clusterChecker->getChildren()->tracks)
						{
							if (childChecker->type == child->type && childChecker->observedId == child->observedId)
							{
								ignore = true;
							}
						}
					}
				}
				TreePtr subTree;
				if (ignore == false)
				{
					LogHelper::logMessage("Building subtree for " + child->type, LOG_DEBUG, LOG_COLOR_TREE);
					subTree = TreePtr(new Tree(pattern, clusterId, child, clustersLeft, allClusters, ism, naive));
				}
				else
				{
					std::vector<ClusterPtr> dummyClusters;
					LogHelper::logMessage("Would build subtree for " + child->type
							+ " but must not. Appending as leaf instead", LOG_DEBUG, LOG_COLOR_TREE);
					subTree = TreePtr(new Tree(pattern, clusterId, child, dummyClusters, allClusters, ism, naive));
				}
				std::vector<VoteSpecifierPtr> votes =
					cluster->getVotersByVotersTypeAndObservedId(child->type, child->observedId);

				this->addChild(subTree);
				if (subTree->getHeight() + 1 > this->getHeight())
				{
					this->setHeight(subTree->getHeight() + 1);
				}
                for (VoteSpecifierPtr& vote : votes)
				{
					vote->patternName = this->getReference()->type;
					vote->observedId = child->observedId;
					vote->objectType = subTree->getReference()->type;
					ism->voteSpecifiersPerObject[subTree->getReference()->type].push_back(vote);
				}
				if (subTree->getWeight() > 1)
				{
					++clusterId;
				}
				this->setWeight(this->getWeight() + subTree->getWeight());
			}
			PatternPtr newPattern(new Pattern(this->getReference()->type, this->getWeight()));
			ism->patternDefinitions.insert(std::make_pair(newPattern->name, newPattern));
		}
		ism->objectTypes.insert(this->getReference()->type);
		LogHelper::logMessage("BackTrack. Weight of reference is "
				+ std::to_string(this->getWeight())
				+ " and height is " + std::to_string(this->getHeight())
				, LOG_DEBUG, LOG_COLOR_TREE);
	}

	std::vector<ClusterPtr> Tree::buildClusters(ISM::ObjectRelations topology) const
	{

		LogHelper::logLine(LOG_DEBUG);
		LogHelper::logMessage("Starting topology clustering : \n", LOG_DEBUG, LOG_COLOR_CLUSTER);
		std::vector<ClusterPtr> clusters;
		std::vector<std::pair<TrackPtr, ClusterPtr> > referencesWithParent;
		//objectOccurenceInfo.first is the amount of occurence of a track, .second is a vector ordered by occurence
		std::pair<std::map<TrackPtr, int>, std::vector<std::pair<int, TrackPtr> > > objectOccurenceInfo = getMostCommonObjects(topology);
		std::map<TrackPtr, int> objectOccurences = objectOccurenceInfo.first;
		std::vector<std::pair<int, TrackPtr> > mostCommonObjects = objectOccurenceInfo.second;
		ClusterPtr sentinel;
		referencesWithParent.push_back(std::make_pair(mostCommonObjects.begin()->second, sentinel));

		LogHelper::logMessage("Starting at most common object : " + (*referencesWithParent.begin()).first->type, LOG_DEBUG, LOG_COLOR_CLUSTER);
		for (size_t tracksIt = 0; tracksIt < referencesWithParent.size(); ++tracksIt)
		{
            for (const std::pair<TrackPtr, int>& it : objectOccurences)
			{
                assert(it.second >= 0); (void)it;
			}
			ClusterPtr currentCluster(new Cluster(std::make_pair(TrackPtr(new Track(*referencesWithParent[tracksIt].first)), referencesWithParent[tracksIt].second), topology, mostCommonObjects, objectOccurences));

			std::ostringstream s;
			s.str("");
			s << "Current cluster contains "  << currentCluster->getChildren()->tracks.size() << " objects: "<< std::endl;
            for (TrackPtr trackIt : currentCluster->getChildren()->tracks)
			{
				s << trackIt->type << " with ID " << trackIt->observedId << ", ";
			}
			s << std::endl << "Reference is : " << currentCluster->getReference()->type << std::endl;
			LogHelper::logMessage(s.str(), LOG_DEBUG, LOG_COLOR_CLUSTER);

			if (currentCluster->getChildren()->tracks.size() > 0)
			{
				clusters.push_back(currentCluster);
			}

            for (const std::pair<TrackPtr, int>& objectIt : objectOccurences)
			{
				if (objectIt.second > 0)
				{
					std::pair<ISM::TrackPtr, ClusterPtr> subReference = getReference(mostCommonObjects, clusters, topology);
					assert (subReference.first != 0);
					referencesWithParent.push_back(subReference);
					break;
				}
			}
		}
        for (const std::pair<TrackPtr, int>& it : objectOccurences)
		{
            assert(it.second == 0); (void)it;
		}

		LogHelper::logMessage("All clusters have been constructed", LOG_DEBUG, LOG_COLOR_CLUSTER);
		LogHelper::logLine(LOG_DEBUG);

		return clusters;
	}

	std::pair<std::map<ISM::TrackPtr, int> , std::vector<std::pair<int, ISM::TrackPtr> > >
		Tree::getMostCommonObjects(const ISM::ObjectRelations& topology) const
	{
		std::map<ISM::TrackPtr, int> objectOccurences;
		std::vector<std::pair<int, ISM::TrackPtr > > mostCommon;
        for (const std::pair<unsigned int, ObjectRelationPtr> relationIt : topology)
		{
			TrackPtr objectA = relationIt.second->getTrackA();
			TrackPtr objectB = relationIt.second->getTrackB();
			if (objectOccurences[objectA])
			{
				++objectOccurences[objectA];
			}
			else
			{
				objectOccurences[objectA] = 1;
			}
			if (objectOccurences[objectB])
			{
				++objectOccurences[objectB];
			}
			else
			{
				objectOccurences[objectB] = 1;
			}
		}
		//Pushing elements to vektor of pair to make it sorted
        for (const std::pair<TrackPtr, int>& it : objectOccurences)
		{
			mostCommon.push_back(std::make_pair(it.second, it.first));
		}

		//Sort mostCommon by number of occurrences of the object and by the object identifier (object type + object observedId).
		struct compare {
			bool operator()(const std::pair<int, ISM::TrackPtr > &firstElem, const std::pair<int, ISM::TrackPtr > &secondElem) {
				if (firstElem.first == secondElem.first)
				{
					std::string objectIdentifierFirstElem = firstElem.second->type + firstElem.second->observedId;
					std::string objectIdentifierSecondElem = secondElem.second->type + secondElem.second->observedId;
					return objectIdentifierFirstElem.compare(objectIdentifierSecondElem) < 0;

				}
				return firstElem.first > secondElem.first;
			}
		};

		std::sort(mostCommon.begin(), mostCommon.end(), compare());
        std::pair<std::map<TrackPtr, int>, std::vector<std::pair<int, ISM::TrackPtr > > > ret = std::make_pair(objectOccurences, mostCommon);
		return ret;
	}

	std::pair<TrackPtr, ClusterPtr> Tree::getReference(std::vector<std::pair<int, TrackPtr> >& objectTracks,
			const std::vector<ClusterPtr>& currentClusters, const ObjectRelations& topology) const
	{
		std::vector<TrackPtr> orderedObjectsInRelations;
        for (std::pair<int, TrackPtr>& tracksIt : objectTracks)
		{
            for (const std::pair<unsigned int, ObjectRelationPtr>& relationIt : topology)
			{
				if (relationIt.second->containsObject(tracksIt.second->type, tracksIt.second->observedId))
				{
					orderedObjectsInRelations.push_back(tracksIt.second);
				}
			}
		}

		ClusterPtr recommendedParent;
		TrackPtr reference = *orderedObjectsInRelations.begin();
        for (TrackPtr& tracksIt : orderedObjectsInRelations)
		{
			if (recommendedParent != 0)
			{
				break;
			}
            for (const ClusterPtr& clusterIt : currentClusters)
			{
				if (recommendedParent != 0)
				{
					LogHelper::logMessage("Comparing with recommended parent :", LOG_DEBUG, LOG_COLOR_TREE);
					LogHelper::logMessage(std::to_string(recommendedParent->getRecommendedLevel()) + " vs " + std::to_string(clusterIt->getRecommendedLevel()), LOG_DEBUG);
				}
				if (recommendedParent != 0 && recommendedParent->getRecommendedLevel() < clusterIt->getRecommendedLevel())
				{
					continue;
				}
                for (TrackPtr& childIt : clusterIt->getChildren()->tracks)
				{
					if ((childIt->observedId == tracksIt->observedId) && (childIt->type  == tracksIt->type))
					{
						assert (tracksIt != 0);
						recommendedParent = clusterIt;
						reference = tracksIt;
					}
				}
			}
		}
		LogHelper::logMessage("Return reference with parent", LOG_DEBUG, LOG_COLOR_CLUSTER);
		return std::make_pair(reference, recommendedParent);
	}

	bool Tree::compareClusters(const ClusterPtr& one, const ClusterPtr& two)
	{
		return one->getRecommendedLevel() < two->getRecommendedLevel();
	}

	template <typename T> bool Tree::pairCompare(const std::pair<unsigned, T>& firstElem,
			const std::pair<unsigned, T>& secondElem)
	{
		return firstElem.first > secondElem.first;
	}

	void Tree::addChild(const boost::shared_ptr<Tree>& child)
	{
		this->children.push_back(child);
	}

	void Tree::setHeight(unsigned height)
	{
		this->height = height;
	}

	unsigned Tree::getHeight()
	{
		return this->height;
	}

	void Tree::setWeight(unsigned weight)
	{
		this->weight = weight;
	}

	unsigned Tree::getWeight()
	{
		return this->weight;
	}

	void Tree::addVotes(const std::string& voterType, const std::vector<VoteSpecifierPtr>& votes)
	{
        std::vector<VoteSpecifierPtr> copy = votes;
		this->votesByVotersType.insert(std::make_pair(voterType, votes));
	}

	std::vector<VoteSpecifierPtr> Tree::getVotesByType(const std::string& type)
	{
		return this->votesByVotersType.at(type);
	}

	TrackPtr Tree::getReference()
	{
		return this->reference;
	}

	IsmPtr Tree::getISM()
	{
		return this->ism;
	}

	void Tree::setISM(IsmPtr ism)
	{
		this->ism = ism;
	}
}

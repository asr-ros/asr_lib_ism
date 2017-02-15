/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Cluster.hpp"
#include "utility/Util.hpp"
#include "utility/LogHelper.hpp"

namespace ISM
{
	Cluster::Cluster(const std::pair<TrackPtr, ClusterPtr>& referenceWithParent,
			ObjectRelations& topology, const std::vector<std::pair<int, TrackPtr> >& mostCommonObjects,
			std::map<TrackPtr, int>& objectOccurences)
	{
		if (referenceWithParent.second == 0)
		{
			this->recommendedLevel = 0;
		}
		else
		{
			this->recommendedLevel = referenceWithParent.second->getRecommendedLevel() + 1;
		}

		this->reference = referenceWithParent.first;
		std::vector<TrackPtr> clustersTracks;
		this->children = TracksPtr(new Tracks(clustersTracks));
		//Iterate over relations to check whether the current Object is part of the current relation.
		//If so, insert the track into the cluster and erase the current relation
		bool referenceInserted = false;
        for (ObjectRelations::iterator relationIt = topology.begin(); relationIt != topology.end();)
		{
			bool erased = false;
			if (relationIt->second->containsObject(referenceWithParent.first->type,
						referenceWithParent.first->observedId))
			{
				bool childInserted = false;
                for (std::vector<std::pair<int, TrackPtr> >::const_iterator tracksIt = mostCommonObjects.begin();
						tracksIt != mostCommonObjects.end();
						++tracksIt)
				{
					if (tracksIt->second->type == referenceWithParent.first->type &&
							tracksIt->second->observedId == referenceWithParent.first->observedId)
					{
						if (referenceInserted == false)
						{
							this->children->tracks.push_back(TrackPtr(new Track(*(*tracksIt).second)));
							referenceInserted = true;
							std::vector<VoteSpecifierPtr> votes;

							for (size_t i = 0; i < referenceWithParent.first->objects.size(); ++i)
							{
								if (referenceWithParent.first->objects[i] == 0)
								{
									//Skips those snapshots where at least one of the two objects isn't present at all
									continue;
								}
								ISM::QuaternionPtr q = ISM::QuaternionPtr(new ISM::Quaternion(1,0,0,0));
								double r = 0;
								VoteSpecifierPtr vote = VoteSpecifierPtr(new VoteSpecifier(q,q,q,q,r));
								vote->trackIndex = i;
								votes.push_back(vote);
							}
							if (votesByVotersTypeAndObservedId.find(referenceWithParent.first->type) ==
									votesByVotersTypeAndObservedId.end())
							{
								votesByVotersTypeAndObservedId[referenceWithParent.first->type] =
									std::map<std::string, std::vector<VoteSpecifierPtr>>();
							}
							votesByVotersTypeAndObservedId[referenceWithParent.first->type][referenceWithParent.first->observedId] = votes;
						}
					}
					else if (relationIt->second->containsObject(tracksIt->second->type, tracksIt->second->observedId))
					{
						this->children->tracks.push_back(TrackPtr(new Track(*(*tracksIt).second)));
						childInserted = true;
						std::vector<VoteSpecifierPtr> votes;
						if (relationIt->second->getObjectTypeA() == tracksIt->second->type &&
								relationIt->second->getObjectIdA() == tracksIt->second->observedId)
						{
							votes = relationIt->second->getVotesFromAForReferencePoseB();
						}
						else
						{
							votes = relationIt->second->getVotesFromBForReferencePoseA();
						}
						if (votesByVotersTypeAndObservedId.find(tracksIt->second->type) ==
								votesByVotersTypeAndObservedId.end())
						{
							votesByVotersTypeAndObservedId[tracksIt->second->type] =
								std::map<std::string, std::vector<VoteSpecifierPtr>>();
						}
						votesByVotersTypeAndObservedId[tracksIt->second->type][tracksIt->second->observedId] = votes;
					}

					if (childInserted && referenceInserted)
					{
						break;
					}
				}

				objectOccurences[relationIt->second->getTrackA()] -= 1;
				objectOccurences[relationIt->second->getTrackB()] -= 1;

				topology.erase(relationIt++);
				erased = true;
			}
			if (!erased)
			{
				++relationIt;
			}
		}
	}

	std::vector<VoteSpecifierPtr> Cluster::getVotersByVotersTypeAndObservedId(const std::string& type,
			const std::string &observedId) const
	{
		return this->votesByVotersTypeAndObservedId.at(type).at(observedId);
	}

	unsigned Cluster::getRecommendedLevel()
	{
		return this->recommendedLevel;
	}
}

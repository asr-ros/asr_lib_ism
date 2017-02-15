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
#include <boost/shared_ptr.hpp>

#include "typedef.hpp"
#include "Cluster.hpp"
#include "common_type/Track.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "ImplicitShapeModel.hpp"

#include "utility/LogHelper.hpp"

namespace ISM
{

class Tree
{
	public:
		Tree(TrackPtr reference) : reference(reference), height(0), weight(1)
		{
			this->reference = TrackPtr(new Track(*reference));
		}

		Tree(const std::string &pattern, ObjectRelations topology, bool naive = false);
		void addChild(const boost::shared_ptr<Tree>& child);
		void setHeight(unsigned height);
		unsigned getHeight();
		void setWeight(unsigned weight);
		unsigned getWeight();
		void addVotes(const std::string& voterType, const std::vector<VoteSpecifierPtr>& votes);
		std::vector<VoteSpecifierPtr> getVotesByType(const std::string& type);
		TrackPtr getReference();
		IsmPtr getISM();
		void setISM(IsmPtr ism);

	private:
		Tree(std::string pattern, unsigned clusterId, const TrackPtr reference, std::vector<ClusterPtr>& clustersLeft, const std::vector<ClusterPtr>& allClusters, IsmPtr& ism, bool naive);

		void build(std::string pattern, unsigned clusterId, const TrackPtr reference, std::vector<ClusterPtr>& clustersLeft, const std::vector<ClusterPtr>& allClusters, IsmPtr& ism, bool naive);
		TreePtr generateTree(const std::string& pattern, const TrackPtr& reference, std::vector<ClusterPtr> clusters, bool naive = false);
		std::vector<VoteSpecifierPtr> generateISM(ISM::ObjectRelations);
		std::vector<ClusterPtr> buildClusters(ISM::ObjectRelations) const;
		std::pair<std::map<ISM::TrackPtr, int> , std::vector<std::pair<int, ISM::TrackPtr > > > getMostCommonObjects(const ISM::ObjectRelations& topology) const;
		std::pair<TrackPtr, ClusterPtr> getReference(std::vector<std::pair<int, TrackPtr> >& objectTracks, const std::vector<ClusterPtr>& currentClusters, const ObjectRelations& topology) const;
		static bool compareClusters(const ClusterPtr& one, const ClusterPtr& two);
		template <typename T> static bool pairCompare(const std::pair<unsigned, T>& firstElem, const std::pair<unsigned, T>& secondElem);


		const char* LOG_COLOR_TREE = LogHelper::LOG_COLOR_GREEN;
		const char* LOG_COLOR_CLUSTER = LogHelper::LOG_COLOR_MAGENTA;

		TrackPtr reference;
		std::vector<boost::shared_ptr<Tree> > children;
		unsigned height;
		unsigned weight;
		std::map<std::string, std::vector<VoteSpecifierPtr> > votesByVotersType;
		IsmPtr ism;

};

}

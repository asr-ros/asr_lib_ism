/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <stdlib.h>

#include "ObjectRelation.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "utility/GeometryHelper.hpp"
#include "common_type/Quaternion.hpp"

namespace ISM
{
	ObjectRelation::ObjectRelation(ISM::TrackPtr objectA, ISM::TrackPtr objectB, std::string patternName)
	{
		this->votesFromBForReferencePoseA = std::vector<VoteSpecifierPtr>();
		this->votesFromAForReferencePoseB = std::vector<VoteSpecifierPtr>();
		if (objectA == 0 || objectB == 0)
		{
			std::cerr<<"invalid tracks\n";
			exit(EXIT_FAILURE);
		}
		for (size_t i = 0; i < objectA->objects.size() && i < objectB->objects.size(); ++i)
		{
			//Keep in mind that the reference object gets voted for, so voteBToA actually refers
			//to the relative Positions of B in relation to A which is why voteBToA is
			//inserted to aToB. aToB means: where might B be from A point of view
			if (objectA->objects[i] == 0 || objectB->objects[i] == 0)
			{
				//Skips those snapshots where at least one of the two objects isn't present at all
				continue;
			}
			VoteSpecifierPtr voteFromBForReferencePoseA =
                GeometryHelper::createVoteSpecifier(objectB->objects[i]->pose,
                        objectA->objects[i]->pose);

            assert(GeometryHelper::poseEqual(GeometryHelper::getPoseFromVote(objectB->objects[i]->pose,
							voteFromBForReferencePoseA),
                        objectA->objects[i]->pose));

			voteFromBForReferencePoseA->patternName = patternName;
			voteFromBForReferencePoseA->observedId = objectB->observedId;
			voteFromBForReferencePoseA->objectType = objectB->type;
			voteFromBForReferencePoseA->trackIndex = i;
			voteFromBForReferencePoseA->weight = 0;
			this->votesFromBForReferencePoseA.push_back(voteFromBForReferencePoseA);

			VoteSpecifierPtr voteFromAForReferencePoseB =
                GeometryHelper::createVoteSpecifier(objectA->objects[i]->pose,
                        objectB->objects[i]->pose);

            assert(GeometryHelper::poseEqual(GeometryHelper::getPoseFromVote(objectA->objects[i]->pose,
							voteFromAForReferencePoseB),
                        objectB->objects[i]->pose));

			voteFromAForReferencePoseB->patternName = patternName;
			voteFromAForReferencePoseB->observedId = objectA->observedId;
			voteFromAForReferencePoseB->objectType = objectA->type;
			voteFromAForReferencePoseB->trackIndex = i;
			voteFromAForReferencePoseB->weight = 0;
			this->votesFromAForReferencePoseB.push_back(voteFromAForReferencePoseB);
		}
		this->objectIdA = objectA->observedId;
		this->objectIdB = objectB->observedId;
		this->objectTypeA = objectA->type;
		this->objectTypeB = objectB->type;
		this->trackA = objectA;
		this->trackB = objectB;
	}

	ObjectRelation::ObjectRelation(ISM::TrackPtr objectA, std::string patternName)
	{
		this->votesFromBForReferencePoseA = std::vector<VoteSpecifierPtr>();
		this->votesFromAForReferencePoseB = std::vector<VoteSpecifierPtr>();
		if (objectA == 0)
		{
			std::cerr<<"invalid tracks\n";
			exit(EXIT_FAILURE);
		}

		for (size_t i = 0; i < objectA->objects.size(); ++i)
		{
			//Keep in mind that the reference object gets voted for, so voteBToA actually refers
			//to the relative Positions of B in relation to A which is why voteBToA is
			//inserted to aToB. aToB means: where might B be from A point of view
			if (objectA->objects[i] == 0)
			{
				//Skips those snapshots where at least one of the two objects isn't present at all
				continue;
			}
			//Identity quaternion. No transformation to self is needed
			ISM::QuaternionPtr q = ISM::QuaternionPtr(new ISM::Quaternion(1,0,0,0));
			double r = 0;
			VoteSpecifierPtr voteFromBForReferencePoseA = VoteSpecifierPtr(new VoteSpecifier(q,q,q,q,r));
			voteFromBForReferencePoseA->patternName = patternName;
			voteFromBForReferencePoseA->observedId = objectA->observedId;
			voteFromBForReferencePoseA->objectType = objectA->type;
			voteFromBForReferencePoseA->trackIndex = i;
			voteFromBForReferencePoseA->weight = 0;
			this->votesFromBForReferencePoseA.push_back(voteFromBForReferencePoseA);

			VoteSpecifierPtr voteFromAForReferencePoseB = VoteSpecifierPtr(new VoteSpecifier(q,q,q,q,r));
			voteFromAForReferencePoseB->patternName = patternName;
			voteFromAForReferencePoseB->observedId = objectA->observedId;
			voteFromAForReferencePoseB->objectType = objectA->type;
			voteFromAForReferencePoseB->trackIndex = i;
			voteFromAForReferencePoseB->weight = 0;
			this->votesFromAForReferencePoseB.push_back(voteFromAForReferencePoseB);
		}
		this->objectIdA = objectA->observedId;
		this->objectIdB = objectA->observedId;
		this->objectTypeA = objectA->type;
		this->objectTypeB = objectA->type;
		this->trackA = objectA;
		this->trackB = objectA;
	}

	std::string ObjectRelation::getObjectIdA() const
	{
		return objectIdA;
	}

	std::string ObjectRelation::getObjectIdB() const
	{
		return objectIdB;
	}

	std::string ObjectRelation::getObjectTypeA() const
	{
		return objectTypeA;
	}

	std::string ObjectRelation::getObjectTypeB() const
	{
		return objectTypeB;
	}

	std::vector<ISM::VoteSpecifierPtr> ObjectRelation::getVotesFromAForReferencePoseB() const
	{
		return this->votesFromAForReferencePoseB;
	}

	std::vector<ISM::VoteSpecifierPtr> ObjectRelation::getVotesFromBForReferencePoseA() const
	{
		return this->votesFromBForReferencePoseA;
	}

	bool ObjectRelation::containsObject(const std::string& type, const std::string& id)
	{
		return ((type == getObjectTypeA() && id == getObjectIdA()) ||
				(type == getObjectTypeB() && id == getObjectIdB()));
	}

	std::ostream& operator<<(std::ostream &strm, const ISM::ObjectRelation &r) {
		return strm << r.getObjectTypeA() << " (ID: " << r.getObjectIdA()<<") <-> "
			<< r.getObjectTypeB() << " (ID: " << r.getObjectIdB()<<")";
	}

	std::ostream& operator<<(std::ostream &strm, const ISM::ObjectRelationPtr &r) {
		return strm<<(*r);
	}

	std::ostream& operator<<(std::ostream &strm, const ISM::ObjectRelations &r) {
		ISM::ObjectRelations relations = r;
		std::ostringstream os;
		strm << "Topology [";
		for (ObjectRelations::iterator it = relations.begin(); it != relations.end();)
		{
			strm << it->first;
			os << "\t- Relation " << it->first << " : " << it->second  << std::endl;

			++it;
			if (it != relations.end())
			{
				strm << ", ";
			} else {
				os << std::endl;
			}
		}
		strm << "] with relations : " << std::endl << os.str();
		return strm;
	}

	void ObjectRelation::serialize(std::ostream& strm) const {
		strm << "{\"typeA\": " << this->getObjectTypeA()
			<< ", \"idA\": " <<this->getObjectIdA()
			<< ", \"typeB\": " << this->getObjectIdB()
			<< ", \"idB\": " << this->getObjectIdB() << "}";
	}

}

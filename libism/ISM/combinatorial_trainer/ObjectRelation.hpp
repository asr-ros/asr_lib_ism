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
#include <map>
#include <functional>
#include "common_type/Serializable.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "common_type/Quaternion.hpp"
#include "common_type/Track.hpp"

namespace ISM
{
/*
 *Class to store relations between two objects.
 */

class ObjectRelation : public Serializable
{
	public:
		ObjectRelation (ISM::TrackPtr objectA, ISM::TrackPtr objectB, std::string patternName);
		ObjectRelation (ISM::TrackPtr objectA, std::string patternName);
		//ObjectRelation (const ISM::ObjectRelation& other);
		//std::pair<std::string, std::string> getObjectIds();
		std::string getObjectIdA() const;
		std::string getObjectIdB() const;
		//std::pair<std::string, std::string> getObjectTypes();
		std::string getObjectTypeA() const;
		std::string getObjectTypeB() const;
		//std::pair<std::vector<ISM::VoteSpecifierPtr>, std::vector<ISM::VoteSpecifierPtr> > getVoteSpecifiers();
		std::vector<ISM::VoteSpecifierPtr> getVotesFromAForReferencePoseB() const;
		std::vector<ISM::VoteSpecifierPtr> getVotesFromBForReferencePoseA() const;
		TrackPtr getTrackA()
		{
			return trackA;
		}
		TrackPtr getTrackB()
		{
			return trackB;
		}
		bool containsObject(const std::string& type, const std::string& id);
		virtual void serialize(std::ostream& strm) const;

	private:
		//The two objects whichs positions relative to each other, are stored
		std::string objectIdA;
		std::string objectIdB;
		std::string objectTypeA;
		std::string objectTypeB;
		TrackPtr trackA;
		TrackPtr trackB;
		//The valid positions B might have if A is reference
		std::vector<VoteSpecifierPtr> votesFromBForReferencePoseA;
		//The valid positions A might have if B is reference
		std::vector<VoteSpecifierPtr> votesFromAForReferencePoseB;
};

typedef boost::shared_ptr<ISM::ObjectRelation> ObjectRelationPtr;
typedef std::map<unsigned int, ISM::ObjectRelationPtr, std::less<unsigned> >ObjectRelations;
std::ostream& operator<<(std::ostream &strm, const ISM::ObjectRelation &q);
std::ostream& operator<<(std::ostream &strm, const ISM::ObjectRelationPtr &q);
std::ostream& operator<<(std::ostream &strm, const ISM::ObjectRelations &q);

}

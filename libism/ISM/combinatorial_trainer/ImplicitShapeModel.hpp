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
#include <set>
#include <map>

#include "common_type/Pattern.hpp"
#include "common_type/VoteSpecifier.hpp"

namespace ISM
{

using std::endl;

struct ImplicitShapeModel
{
	std::map<std::string, PatternPtr> patternDefinitions;
	std::map<std::string, std::vector<VoteSpecifierPtr> > voteSpecifiersPerObject;
	std::set<std::string> objectTypes;

	ImplicitShapeModel() {};
	ImplicitShapeModel(std::map<std::string, PatternPtr> patternDefinitions,
			std::map<std::string, std::vector<VoteSpecifierPtr> > voteSpecifiersPerObject,
			std::set<std::string> objectTypes) : patternDefinitions(patternDefinitions),
	voteSpecifiersPerObject(voteSpecifiersPerObject),
	objectTypes(objectTypes) {}

	static bool sortVoteSpecPtrAscAlpha(const VoteSpecifierPtr& lhs, const VoteSpecifierPtr& rhs)
	{
		std::stringstream ssl;
		std::stringstream ssr;
		ssl << lhs;
		ssr << rhs;
		return ssl.str() < ssr.str();
	}

	std::string toString()
	{
		std::stringstream strm;
		strm << "PatternDefinitions:" << std::endl;
		for(auto& strPatPair : patternDefinitions)
		{
			strm << strPatPair.first << " : " << strPatPair.second << endl;
		}
		strm << endl;

		strm << "VoteSpecifiersPerObject:" << std::endl;
		for(auto& strVsVecPair : voteSpecifiersPerObject)
		{
			strm << strVsVecPair.first << " :" << endl;
			std::vector<VoteSpecifierPtr> vs = strVsVecPair.second;
			std::sort(vs.begin(), vs.end(), sortVoteSpecPtrAscAlpha);
			for(auto& v : vs)
			{
				strm << v << endl;
			}
			strm << endl;
		}
		strm << endl;


		strm << "ObjectTypes:" << std::endl;
		for(auto& ot : objectTypes)
		{
			strm << ot << endl;
		}
		strm << endl;

		return strm.str();
	}

}; typedef boost::shared_ptr<ImplicitShapeModel> IsmPtr;

}

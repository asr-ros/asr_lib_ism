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

#include "Object.hpp"
#include "Quaternion.hpp"
#include <boost/shared_ptr.hpp>
#include <string>

namespace ISM {
  
    /**
     * VoteSpecifier class. Datastructure that represents a vote. Entries of ism table are votes. Votes are made up of:
     */
    struct VoteSpecifier
    {
        ///Transformation from object in scene to ism reference: Position as direction vector.
        QuaternionPtr objectToRefQuat;
        ///Transformation from object in scene to ism reference: Transforms orientation of object into orientation of reference.
        QuaternionPtr objectToRefPoseQuat;
        ///Transformation from ism reference to object in scene: Position as direction vector.
        QuaternionPtr refToObjectQuat;
        ///Transformation from ism reference to object in scene: Transforms orientation of reference into orientation of object.
        QuaternionPtr refToObjectPoseQuat;
        ///Length of direction vectors. Extracted to make scene representation independant of scale (absolute distances between objects).
        double radius;
        ///Name of scene for which is voted.
        std::string patternName;
        ///Object that votes. Its type.
        std::string objectType;
        ///Object that votes. Its identifier within its class.
        std::string observedId;
        ///Index of the sample, of the voting object's trajectory, from which the vote was generated.
        int trackIndex;
	///weight (eg after filtering)
	uint32_t weight;

        VoteSpecifier( const VoteSpecifier& other ) : radius(other.radius), patternName(other.patternName),
                                                      objectType(other.objectType), observedId(other.observedId),
                                                      trackIndex(other.trackIndex)
        {
            objectToRefQuat = QuaternionPtr(new Quaternion(*(other.objectToRefQuat)));
            objectToRefPoseQuat = QuaternionPtr(new Quaternion(*(other.objectToRefPoseQuat)));
            refToObjectQuat = QuaternionPtr(new Quaternion(*(other.refToObjectQuat)));
            refToObjectPoseQuat = QuaternionPtr(new Quaternion(*(other.refToObjectPoseQuat)));
	    }

        VoteSpecifier( QuaternionPtr objectToRefQuat, QuaternionPtr objectToRefPoseQuat,
                       QuaternionPtr refToObjectQuat, QuaternionPtr refToObjectPoseQuat,
                       double radius ) : objectToRefQuat(objectToRefQuat), objectToRefPoseQuat(objectToRefPoseQuat),
                                         refToObjectQuat(refToObjectQuat), refToObjectPoseQuat(refToObjectPoseQuat),
                                         radius(radius)
        {
            this->trackIndex = -1;
        };

        VoteSpecifier( QuaternionPtr objectToRefQuat, QuaternionPtr objectToRefPoseQuat,
                       QuaternionPtr refToObjectQuat, QuaternionPtr refToObjectPoseQuat,
                       double radius, std::string patternName,std::string objectType,
                       std::string observedId, int trackIndex = -1) : objectToRefQuat(objectToRefQuat), objectToRefPoseQuat(objectToRefPoseQuat),
                                                  refToObjectQuat(refToObjectQuat), refToObjectPoseQuat(refToObjectPoseQuat),
                                                  radius(radius), patternName(patternName), objectType(objectType), observedId(observedId),
                                                  trackIndex(trackIndex)
        {};
    };
    typedef boost::shared_ptr<VoteSpecifier> VoteSpecifierPtr;

    std::ostream& operator<<(std::ostream &strm, const ISM::VoteSpecifier &v);
    std::ostream& operator<<(std::ostream &strm, const ISM::VoteSpecifierPtr &v);
}

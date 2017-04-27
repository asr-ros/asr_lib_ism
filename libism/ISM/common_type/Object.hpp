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

#include <string>
#include <iostream>
#include <sstream>
#include "Pose.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include "Serializable.hpp"

namespace ISM {

/**
     * Object class. Representation of object estimations in training data or input objects.
     */
class Object: public Serializable
{
    public:
        //All information about an object that results from estimation through object localization.
        PosePtr pose;
        std::string type;
        std::string observedId;
        //Path to the mesh, used to visualize the estimated object.
        boost::filesystem::path ressourcePath;
        //Unnormalized and normalized rating how well this object estimate matches the searched scene.
        double weight;
        double confidence;
        std::string providedBy;

        Object()
        {
            this->pose = PosePtr(new Pose());
        };

        Object(const Object& other) : type(other.type), observedId(other.observedId), ressourcePath(other.ressourcePath),
                      weight(other.weight), confidence(other.confidence), providedBy(other.providedBy)
        {
            this->pose = PosePtr(new Pose(*other.pose));
        };

  Object(const std::string type, const std::string observedId = "", const std::string ressourcePath = "", std::string providedBy = "") : type(type), observedId(observedId), ressourcePath(ressourcePath), weight(1), confidence(1), providedBy(providedBy)
        {
            this->pose = ISM::PosePtr(new ISM::Pose());
        };

        Object(const std::string type, PosePtr pose, const std::string observedId = "", const std::string ressourcePath = "", std::string providedBy = "") :
	  pose(pose), type(type), observedId(observedId), ressourcePath(ressourcePath),
      weight(1), confidence(1), providedBy(providedBy)
        {};

        Object(std::string type, Pose* pose, std::string observedId = "", const std::string ressourcePath = "", std::string providedBy = "") :
            type(type), observedId(observedId), ressourcePath(ressourcePath),
            weight(1), confidence(1), providedBy(providedBy)
        {
            this->pose = PosePtr(pose);
        };

        void serialize(std::ostream& strm) const;

        inline bool operator<(const Object& rhs) const
        {
            return this->type < rhs.type && this->observedId < rhs.observedId;
        }
};
typedef boost::shared_ptr<Object> ObjectPtr;

std::ostream& operator<<(std::ostream &strm, const ISM::Object &o);
std::ostream& operator<<(std::ostream &strm, const ISM::ObjectPtr &o);
}

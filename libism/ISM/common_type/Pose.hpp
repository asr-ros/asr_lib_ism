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

#include <sstream>
#include <string>
#include <ostream>
#include "Quaternion.hpp"
#include "Point.hpp"
#include "Serializable.hpp"

namespace ISM {

/**
     * Pose class. "Struct" for 6D pose of an object represented as combination of 3D position vector and orientation quaternion. Both are public.
     */
class Pose : public Serializable
{
public:
    PointPtr point;
    QuaternionPtr quat;

    Pose()
    {
        this->point = PointPtr(new Point(0, 0, 0));
        this->quat = QuaternionPtr(new Quaternion(1.0, 0, 0, 0));
    };

    Pose(const Pose& other)
    {
        this->point = PointPtr(new Point(*(other.point)));
        this->quat = QuaternionPtr(new Quaternion(*(other.quat)));
    };

    Pose(Point *p, Quaternion* q)
    {
        this->point = PointPtr(p);
        this->quat = QuaternionPtr(q);
    };

    Pose(Point *p, QuaternionPtr q) : quat(q)
    {
        this->point = PointPtr(p);
    };

    Pose(PointPtr p, Quaternion* q) : point(p)
    {
        this->quat = QuaternionPtr(q);
    };

    Pose(PointPtr p, QuaternionPtr quat) : point(p), quat(quat) {};

    virtual void serialize(std::ostream& strm) const;
};
typedef boost::shared_ptr<Pose> PosePtr;

bool operator==(const PosePtr& pose1, const PosePtr& pose2);
bool operator==(const Pose& pose1, const Pose& pose2);

std::ostream& operator<<(std::ostream &strm, const ISM::Pose &p);
std::ostream& operator<<(std::ostream &strm, const ISM::PosePtr &p);
}

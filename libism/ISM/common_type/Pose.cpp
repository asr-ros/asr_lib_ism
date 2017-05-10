/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Pose.hpp"
#include "JsonStream.hpp"
#include "utility/GeometryHelper.hpp"

namespace ISM {
  std::ostream& operator<<(std::ostream &strm, const ISM::Pose &p) {
    return strm<<"pose: ["<<p.point<<";"<<p.quat<<"]";
  }

  std::ostream& operator<<(std::ostream &strm, const ISM::PosePtr &p) {
    return strm<<(*p);
  }

  bool operator==(const PosePtr& pose1, const PosePtr& pose2)
  {
      return *pose1 == *pose2;
  }

  bool operator==(const Pose& pose1, const Pose& pose2)
  {
      return pose1.point == pose2.point && pose1.quat == pose2.quat;
  }

  void Pose::serialize(std::ostream& strm) const {
    strm<<"{"<<std::endl<<"\"point\": "<<json(this->point)<<", "<<std::endl
    <<"\"quaternion\": "<<json(this->quat)<<", "<<std::endl
	<<"\"xAxis\": "<<json(GeometryHelper::vectorToPoint(GeometryHelper::getAxisFromQuat(this->quat)))
	<<std::endl<<"}";
  }

  void Pose::convertPoseIntoFrame(const boost::shared_ptr<Pose>& pFrame, boost::shared_ptr<Pose>& pResult)
  {
    // Initialize the result pointer.
    pResult.reset(new Pose());

    // Calculate the relative position by subtracting the position of the child from the parent position.
    // Rotate the resulting relative position into the parent frame.
    pResult->point.reset(new ISM::Point(pFrame->quat->getEigen().toRotationMatrix().inverse() * (point->getEigen() - pFrame->point->getEigen())));

    // The relative orientation is defined as the difference between the orientation of parent and child.
    pResult->quat.reset(new ISM::Quaternion(quat->getEigen() * pFrame->quat->getEigen().inverse()));
  }
}

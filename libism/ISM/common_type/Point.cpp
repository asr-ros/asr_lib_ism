/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Point.hpp"
#include "JsonStream.hpp"
#include "recognizer/VotingSpace.hpp"

#include "math.h"

#include <gtest/internal/gtest-internal.h>

namespace ISM {
    std::ostream& operator<<(std::ostream &strm, const ISM::Point &p) {
      return strm<<"point: ["<<p.eigen.x()<<";"<<p.eigen.y()<<";"<<p.eigen.z()<<"]";
    }

    std::ostream& operator<<(std::ostream &strm, const ISM::PointPtr &p) {
        return strm<<(*p);
    }

    void Point::serialize(std::ostream& strm) const {
      strm<<"{\"x\": "<<this->eigen.x()<<", \"y\": "<<this->eigen.y()<<", \"z\": "<<this->eigen.z()<<"}";
    }

    bool operator==(const PointPtr& p1, const PointPtr& p2)
	{
        return *p1 == *p2;
	}

    bool operator==(const Point& p1, const Point& p2)
	{
    	/*
    	 * this would be the correct way to do it but since pose from vote has inaccuracy
    	 * use threshold
        testing::internal::FloatingPoint<double> x1(p1.eigen.x()), x2(eigen.x());
        testing::internal::FloatingPoint<double> y1(p1.eigen.y()), y2(eigen.y());
        testing::internal::FloatingPoint<double> z1(p1.eigen.z()), z2(eigen.z());
    	return x1.AlmostEquals(x2) && y1.AlmostEquals(y2) && z1.AlmostEquals(z2);
    	*/

    	//threshold determined statistically
    	const double threshold = 1e-15;
        double diffX = fabs(p1.eigen.x() - p2.eigen.x());
        double diffY = fabs(p1.eigen.y() - p2.eigen.y());
        double diffZ = fabs(p1.eigen.z() - p2.eigen.z());
    	return diffX < threshold && diffY < threshold && diffZ < threshold;
	}

    Eigen::Vector3d Point::getEigen() {
        return this->eigen;
    }
}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Quaternion.hpp"

#include <gtest/internal/gtest-internal.h>

namespace ISM {
    std::ostream& operator<<(std::ostream &strm, const ISM::Quaternion &q) {
        return strm<<"quaternion: ["<<q.eigen.w() << ";" << q.eigen.x() <<";" <<q.eigen.y() << ";" << q.eigen.z() << "]";
    }

    std::ostream& operator<<(std::ostream &strm, const ISM::QuaternionPtr &q) {
        return strm<<(*q);
    }

    void Quaternion::serialize(std::ostream& strm) const {
        strm<<"{\"w\": "<<this->eigen.w()<<", \"x\": "<<this->eigen.x()<<", \"y\": "<<this->eigen.y()<<", \"z\": "<<this->eigen.z()<<"}";
    }

    bool operator==(const QuaternionPtr& q1, const QuaternionPtr& q2)
	{
        return *q1 == *q2;
	}

    bool operator==(const Quaternion& q1, const Quaternion& q2)
	{
    	/*
    	 * this would be the correct way to do it but since pose from vote has inaccuracy
    	 * use threshold

        testing::internal::FloatingPoint<double> w1(q.eigen.w()), w2(eigen.w());
        testing::internal::FloatingPoint<double> x1(q.eigen.x()), x2(eigen.x());
        testing::internal::FloatingPoint<double> y1(q.eigen.y()), y2(eigen.y());
        testing::internal::FloatingPoint<double> z1(q.eigen.z()), z2(eigen.z());
    	return w1.AlmostEquals(w2) && x1.AlmostEquals(x2) && y1.AlmostEquals(y2) && z1.AlmostEquals(z2);
    	*/

    	//threshold determined statistically
    	const double threshold = 1e-15;
        double diffW = fabs(q1.eigen.w() - q2.eigen.w());
        double diffX = fabs(q1.eigen.x() - q2.eigen.x());
        double diffY = fabs(q1.eigen.y() - q2.eigen.y());
        double diffZ = fabs(q1.eigen.z() - q2.eigen.z());
    	return diffW < threshold && diffX < threshold && diffY < threshold && diffZ < threshold;
	}
}

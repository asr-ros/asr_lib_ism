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

#include <Eigen/Geometry>
#include <cmath>
#include "common_type/Pose.hpp"
#include "common_type/Quaternion.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "common_type/Point.hpp"

namespace ISM {
  class MathHelper {
  public:

    static bool getNextCombination(const std::vector<unsigned int>::iterator first, std::vector<unsigned int>::iterator k, const std::vector<unsigned int>::iterator last)
    {
      if ((first == last) || (first == k) || (last == k))
	return false;

      std::vector<unsigned int>::iterator it1 = first;
      std::vector<unsigned int>::iterator it2 = last;
      it1++;
      if (last == it1)
	return false;
      it1 = last;
      it1--;
      it1 = k;
      it2--;
      while (first != it1)
        {
	  if (*--it1 < *it2)
            {
	      std::vector<unsigned int>::iterator j = k;
	      while (!(*it1 < *j)) ++j;
	      std::iter_swap(it1,j);
	      it1++;
	      j++;
	      it2 = k;
	      std::rotate(it1,j,last);
	      while (last != j)
                {
		  j++;
		  it2++;
                }
	      std::rotate(k,it2,last);
	      return true;
            }
        }
      std::rotate(first,k,last);
      return false;
    }

  };

}

/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ObjectSet.hpp"
#include "utility/GeometryHelper.hpp"

namespace ISM {
bool sortObjectPtrAscAlpha(const ObjectPtr& lhs, const ObjectPtr& rhs)
    {
	std::stringstream ssl;
	std::stringstream ssr;
	ssl << lhs;
	ssr << rhs;
	return ssl.str() < ssr.str();
}
    std::ostream& operator<<(std::ostream &strm, const ISM::ObjectSet &o) {
        strm<<"object set: ["<<std::endl;
        std::vector<ISM::ObjectPtr> os = o.objects;
        std::sort(os.begin(), os.end(), sortObjectPtrAscAlpha);
        for (ObjectPtr& object : os) {
            strm<<object<<std::endl;
        };
        return strm<<"]";
    }

    std::ostream& operator<<(std::ostream &strm, const ISM::ObjectSetPtr &o) {
        return strm<<(*o);
    }

    void ObjectSet::insert(ObjectPtr o) {
    	if(!o)
    {
    		std::cerr << "ObjectSet::insert not valid pointer" << std::endl;
    		throw std::runtime_error("ObjectSet::insert not valid pointer");
    	}
        o->pose->quat = GeometryHelper::normalize(o->pose->quat);
        objects.push_back(o);
    }
}

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

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "Object.hpp"
#include "Point.hpp"

namespace ISM {

    /**
     * ObjectSet class. Configuration of objects at a certain time step. Training data consists of streams of object sets. Input to scene recognition is an object set.
     */
    struct ObjectSet
    {
        
            ObjectSet() {};
            ObjectSet(const ObjectSet& other)
            {
                for (auto& objectPtr : other.objects)
                {
                    this->objects.push_back(ObjectPtr(new Object(*objectPtr)));
                }
            };


            void insert(ObjectPtr o);

            std::vector<ObjectPtr> objects;

    };
    typedef boost::shared_ptr<ObjectSet> ObjectSetPtr;

    std::ostream& operator<<(std::ostream &strm, const ISM::ObjectSet &o);
    std::ostream& operator<<(std::ostream &strm, const ISM::ObjectSetPtr &o);
}

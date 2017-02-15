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
#include <vector>

#include "../common_type/ObjectSet.hpp"


namespace ISM
{

/**
 *  Represent a tool to merge multiple source databases into one target database.
 */
class RecordedObjectsTransformer
{
    public:    
        void transformRecordedObjects(const std::string& sourceFile, const std::string& targetFile, const std::string& type, const std::string& id,
                                      double px, double py, double pz, double qw, double qx, double qy, double qz);

    private:
        void transformSet(ISM::ObjectSetPtr objectSet, Eigen::Vector3d point, Eigen::Quaterniond quat, Eigen::Vector3d orginalCenter);
        void transformSet(ISM::ObjectSetPtr objectSet, Eigen::Vector3d point, double angle);

        /**
         * @brief getReferenceObject Search for reference object in a set
         * @param oset object set
         * @param type type of reference object
         * @param id ID of reference object
         * @return reference object or nullptr if not found
         */
        ObjectPtr getReferenceObject(ObjectSetPtr oset, const std::string& type, const std::string& id);
};
}

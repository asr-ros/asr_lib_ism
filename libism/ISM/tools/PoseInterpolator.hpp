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
class PoseInterpolator
{
    public:    
        void interpolate(const std::string& sourceFile, const std::string& targetFile, int stepNumber);

    private:
        /**
         * @brief interpolate poses between two given poses
         * @param from Source pose
         * @param to Target pose
         * @param stepNumber Number of poses to be interpolated
         * @return vector of interpolated poses
         */
        std::vector<PosePtr> interpolatePoses(PosePtr from, PosePtr to, int stepNumber);

        /**
         * @brief interpolate poses between two given ObjectSets
         * @param from Source ObjectSet
         * @param to Target ObjectSet
         * @param stepNumber Number of poses to be interpolated
         * @return vector of ObjectSets with interpolated poses
         */
        std::vector<ObjectSetPtr> interpolateSets(const ObjectSetPtr from, const ObjectSetPtr to, const int stepNumber);
};
}

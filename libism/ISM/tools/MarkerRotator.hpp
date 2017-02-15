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

#include "../utility/TableHelper.hpp"
#include "../utility/GeometryHelper.hpp"

#include <string>
#include <vector>
#include <fstream>


namespace ISM
{

/**
 *  Represent a tool to rotate marker of a database.
 */
class MarkerRotator
{
    public:

        /**
         *  Rotate coordinates system of marker by 90 degrees arround the x-axis.
         */
        void rotateMarker(const std::string& sourceFile, const std::string& targetFile)
        {
            // copy source to target, so that only the copy will be modified
            std::ifstream source(sourceFile, std::fstream::binary);
            std::ofstream target(targetFile, std::fstream::trunc|std::fstream::binary);
            target << source.rdbuf();

            TableHelper helper(targetFile);
            std::vector<std::pair<int, ObjectPtr>> objectsWithDbIds = helper.getAllMarkerObjects();
            std::cout << "Objects count: " << objectsWithDbIds.size() << std::endl;
            Eigen::Quaterniond eigenQuat;
            Eigen::Matrix3d A, B, C;
            for (std::pair<int, ObjectPtr> objectPair : objectsWithDbIds)
            {
                ObjectPtr object = objectPair.second;
                std::cout << objectPair.first << ": " << object->type << std::endl;
                eigenQuat =  GeometryHelper::quatToEigenQuat(object->pose->quat);
                A = eigenQuat.toRotationMatrix();
                B = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
                std::cout << B << std::endl;
                C = A * B;
                object->pose->quat = GeometryHelper::eigenQuatToQuat(Eigen::Quaterniond(C));
                helper.updateObjectQuaternion(objectPair.first, object);
            }
        }
};
}
